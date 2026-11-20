/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_canfd.c
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

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/can.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#ifdef CONFIG_RZV2H_CANFD_SOCKET
#  include <net/if.h>
#  include <nuttx/nuttx.h>
#  include <nuttx/net/can.h>
#  include <nuttx/net/netdev_lowerhalf.h>
#  ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
#    include <nuttx/clock.h>
#    include <nuttx/wdog.h>
#  endif
#else
#  include <nuttx/can/can.h>
#endif

#include "rzv2h_canfd.h"
#include "rzv2h_irq.h"
#include "rzv2h_fsp_err.h"
#include "rzv2h_gpio.h"
#include "hardware/rzv2h_pinmap.h"

#include <arch/board/board.h>

#include "r_can_api.h"
#include "r_canfd.h"

#if defined(CONFIG_RZV2H_CANFD0) || defined(CONFIG_RZV2H_CANFD1) || \
    defined(CONFIG_RZV2H_CANFD2) || defined(CONFIG_RZV2H_CANFD3) || \
    defined(CONFIG_RZV2H_CANFD4) || defined(CONFIG_RZV2H_CANFD5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_CANFD_BASE          R_CANFD_BASE
#define RZV2H_CANFD_CLK_HZ        BSP_CFG_CLOCK_CANCLK_HZ

/* AFL register bit definitions. */

#define RZV2H_AFL_ID_EXT        (1UL << 31)  /* GAFLIDE: extended ID mode */

#define RZV2H_AFL_MASK_IDMODE   (1UL << 31)  /* GAFLIDEM: match ID mode */

#define RZV2H_AFLDAE            (1UL << 8)   /* AFL Data Access Enable */
#define RZV2H_AFL_PAGE_SIZE     16           /* AFL entries per page window */

/* AFL ID field masks.
 * Standard IDs use bits[10:0] (11-bit), extended use bits[28:0] (29-bit).
 */

#define RZV2H_AFL_ID_MASK_STD   0x7FF        /* 11-bit standard ID mask */
#define RZV2H_AFL_ID_MASK_EXT   0x1FFFFFFF   /* 29-bit extended ID mask */

/* Value in afl_usage->owner[] for the AFL entries of filter 'num' */

#define RZV2H_AFL_OWNER(num)    ((uint8_t)((num) + 1))  /* 0 = free */

/* AFL entry mask values.
 * When mask = 0, the hardware ignores the corresponding
 * id/frame_type/id_mode reference values (don't care semantics).
 */

/* mask_id: don't care on ID bits */

#define RZV2H_AFL_MASK_ID_NOT_USED          0

/* mask_frame_type: don't care on frame type */

#define RZV2H_AFL_MASK_FRAME_TYPE_NOT_USED  0

/* mask_id_mode: don't care on ID mode */

#define RZV2H_AFL_MASK_NOT_USED             0

/* RX FIFO selection for AFL destination */

#define RZV2H_CANFD_RX_FIFO_NONE    0   /* No RX FIFO selected */

/* RX FIFO type of a channel, from CONFIG_RZV2H_CANFDn_RX_FIFO_TYPE
 * (one value per option of the per-channel Kconfig choice "RX FIFO")
 */

#define RZV2H_CANFD_RX_FIFO_TYPE_RXFIFO      0  /* RX FIFO n, GRECC */
#define RZV2H_CANFD_RX_FIFO_TYPE_CFIFO       1  /* Common FIFO 3n, REC */
#define RZV2H_CANFD_RX_FIFO_TYPE_CFIFO_PRIO  2  /* 3n high + 3n+1 low, REC */

/* Common FIFO slots. Channel n owns Common FIFOs 3n, 3n+1 and 3n+2,
 * called slots 0, 1 and 2 here (slot 2 is not used).
 */

#define RZV2H_CANFD_CFIFO_PER_CH    3  /* Slots per channel */
#define RZV2H_CANFD_CFIFO_SINGLE    0  /* Slot 0 (3n):   single FIFO mode */
#define RZV2H_CANFD_CFIFO_HIGH      0  /* Slot 0 (3n):   CAN_MSGPRIO_HIGH */
#define RZV2H_CANFD_CFIFO_LOW       1  /* Slot 1 (3n+1): everything else */
#define RZV2H_CANFD_CFIFO_IDX(ch, slot) \
  ((ch) * RZV2H_CANFD_CFIFO_PER_CH + (slot))
#define RZV2H_CANFD_CFIFO_CH(cfifo) ((cfifo) / RZV2H_CANFD_CFIFO_PER_CH)

/* RX FIFOs 0-7. Channel n owns RX FIFO n */

#define RZV2H_CANFD_RXFIFO_NUM      8       /* RX FIFO 0-7 */
#define RZV2H_CANFD_RXFIFO_IDX(ch)  (ch)    /* Channel -> its RX FIFO */
#define RZV2H_CANFD_RXFIFO_CH(fifo) (fifo)  /* RX FIFO -> its channel */

/* CFDGAFLP1n.GAFLFDP: bits 0-7 select RX FIFO 0-7, bit 8 + j selects
 * Common FIFO j.
 */

#define RZV2H_AFL_FDP_CFIFO_SHIFT   8

/* CFDGAFLP0n.GAFLSRD0-2 must be 0 so that the GAFLFDP Common FIFO bits
 * select the Common FIFO and not a TX queue.
 */

#define RZV2H_AFL_P0_GAFLSRD_MSK    (R_CANFD_CFDGAFL_P0_GAFLSRD0_Msk | \
                                     R_CANFD_CFDGAFL_P0_GAFLSRD1_Msk | \
                                     R_CANFD_CFDGAFL_P0_GAFLSRD2_Msk)

/* CFDCFSTSn flags are cleared by writing 0b and ignore writing 1b.
 * "Do not use the bit clear instruction to clear this bit. Use the MOV
 * instruction to ensure that only the specified bit is cleared. Other
 * bits remain 1b."
 * Refer to "Common FIFO Status Register n (CFDCFSTSn)" in the HW manual.
 */

#define RZV2H_CANFD_CFSTS_FLAGS     (R_CANFD_CFDCFSTS_CFMLT_Msk    | \
                                     R_CANFD_CFDCFSTS_CFRXIF_Msk   | \
                                     R_CANFD_CFDCFSTS_CFTXIF_Msk   | \
                                     R_CANFD_CFDCFSTS_CFFIF_Msk    | \
                                     R_CANFD_CFDCFSTS_CFOFRXIF_Msk | \
                                     R_CANFD_CFDCFSTS_CFOFTXIF_Msk | \
                                     R_CANFD_CFDCFSTS_CFMOW_Msk)
#define RZV2H_CANFD_CFSTS_CLR(flag) (RZV2H_CANFD_CFSTS_FLAGS & ~(flag))

/* CFDRFSTSn and CFDGERFL flags are cleared the same way.
 * Refer to "RX FIFO Status Register n (CFDRFSTSn)" and "Global Error Flag
 * Register (CFDGERFL)" in the HW manual.
 */

#define RZV2H_CANFD_RFSTS_FLAGS     (R_CANFD_CFDRFSTS_RFMLT_Msk | \
                                     R_CANFD_CFDRFSTS_RFIF_Msk  | \
                                     R_CANFD_CFDRFSTS_RFFIF_Msk)
#define RZV2H_CANFD_RFSTS_CLR(flag) (RZV2H_CANFD_RFSTS_FLAGS & ~(flag))

#define RZV2H_CANFD_GERFL_FLAGS     (R_CANFD_CFDGERFL_DEF_Msk   | \
                                     R_CANFD_CFDGERFL_CMPOF_Msk | \
                                     R_CANFD_CFDGERFL_EEF0_Msk  | \
                                     R_CANFD_CFDGERFL_EEF1_Msk  | \
                                     R_CANFD_CFDGERFL_EEF2_Msk  | \
                                     R_CANFD_CFDGERFL_EEF3_Msk  | \
                                     R_CANFD_CFDGERFL_EEF4_Msk  | \
                                     R_CANFD_CFDGERFL_EEF5_Msk)
#define RZV2H_CANFD_GERFL_CLR(flag) (RZV2H_CANFD_GERFL_FLAGS & ~(flag))

/* Bit timing limits for CANIOC_SET_BITTIMING, the automatic timing and the
 * check at open:
 *   Nominal, CFDCnNCFG: NBRP 1-1024, NSJW 1-128,
 *                       NTSEG1 2-256, NTSEG2 2-128
 *   Data,    CFDCnDCFG: DBRP 1-256, DSJW 1-16,
 *                       DTSEG1 2-32, DTSEG2 2-16
 *   Bit: SS + TSEG1 + TSEG2 = 8-385 Tq nominal, 5-49 Tq data. Refer to
 *   "Bit Timing Conditions" in the HW manual.
 */

#define RZV2H_NBRP_MAX           1024
#define RZV2H_NSJW_MAX           128
#define RZV2H_NTSEG1_MIN         2
#define RZV2H_NTSEG1_MAX         256
#define RZV2H_NTSEG2_MIN         2
#define RZV2H_NTSEG2_MAX         128
#define RZV2H_DBRP_MAX           256
#define RZV2H_DSJW_MAX           16
#define RZV2H_DTSEG1_MIN         2
#define RZV2H_DTSEG1_MAX         32
#define RZV2H_DTSEG2_MIN         2
#define RZV2H_DTSEG2_MAX         16
#define RZV2H_NTQ_MIN            8
#define RZV2H_NTQ_MAX            385
#define RZV2H_DTQ_MIN            5
#define RZV2H_DTQ_MAX            49

/* CFDGCTR register: GMDC[1:0] — Global Mode Control.
 * 11b = no effect on the module operating mode.
 */

#define RZV2H_CANFD_GMDC_NO_CHANGE  0x3

/* TX message buffers. Each channel has 32: TXMB0-15 and TXMB32-47.
 * The driver uses TXMB0 .. CONFIG_RZV2H_CANFDn_TXMB_NUM - 1 only.
 * Kconfig limits CONFIG_RZV2H_CANFDn_TXMB_NUM to 1-16.
 */

/* Global TX message buffer number of TX message buffer 'mb' on channel
 * 'ch'. Each channel owns 64 TX message buffer slots, so the CFDTMCn
 * (TX message buffer control) and CFDTMSTSn (TX message buffer status)
 * register arrays (384 entries each, canfd_iodefine.h) are indexed with
 * mb + ch * 64.
 */

#define RZV2H_CANFD_TXMB_IDX(ch, mb) \
  ((mb) + (ch) * BSP_FEATURE_CANFD_TXMB_CHANNEL_OFFSET)

/* canfd_extended_cfg_t.txmb_txi_enable for TXMB0 .. n-1: bit m enables the
 * TX complete interrupt of TXMB m. R_CANFD_Open() copies the mask into
 * this channel's CFDTMIEC registers (r_canfd.c:477-485).
 */

#define RZV2H_CANFD_TXMB_MASK(n)  ((1ULL << (n)) - 1)

/* Payload bytes of a classic CAN frame */

#define RZV2H_CANFD_CLASSIC_MAXLEN  8

/* Most TX message buffers per channel: range of RZV2H_CANFDn_TXMB_NUM */

#define RZV2H_CANFD_SOCK_TXMB_MAX  16

/* A TX message buffer is busy while either of these bits is 1:
 *   CFDTMSTSn.TMTRM - transmit request pending (mirror of CFDTMCn.TMTR).
 *   CFDTMSTSn.TMTRF - result of the previous request, not yet cleared.
 *                     CFDTMCn.TMTR can only be set when CFDTMSTSn.TMTRF
 *                     is 00b.
 *
 * CFDTMSTSn.TMTSTS and CFDTMSTSn.TMTARM are not checked: each can only be
 * 1 while CFDTMCn.TMTR is set, so CFDTMSTSn.TMTRM already covers them.
 *
 * Refer to "TX Message Buffer Status Register n (CFDTMSTSn)" and
 * "TX Message Buffer Control Register n (CFDTMCn)" in the HW manual.
 */

#define RZV2H_CANFD_TMSTS_BUSY    (R_CANFD_CFDTMSTS_TMTRF_Msk | \
                                   R_CANFD_CFDTMSTS_TMTRM_Msk)

/* Longest wait in microseconds for the TX aborts before bus-off recovery */

#define RZV2H_CANFD_ABORT_WAIT_US 1000

/* Build one CFDRFCC register value from Kconfig.
 * INT_MODE: 0=disable, 1=at threshold (RFIE), 2=every frame (RFIE+RFIM).
 * RFIM: 0b = at the RFIGCV threshold, 1b = every received message.
 * Refer to "RX FIFO Configuration/Control Register n (CFDRFCCn)" in the HW
 * manual.
 */

#define RZV2H_CANFD_RXFIFO_EN(n) \
    (CONFIG_RZV2H_CANFD_RXFIFO##n##_ENABLE ? R_CANFD_CFDRFCC_RFE_Msk : 0)

#define RZV2H_CANFD_RXFIFO_CFG(n)                                    \
  ((CONFIG_RZV2H_CANFD_RXFIFO##n##_INT_THRESHOLD                     \
      << R_CANFD_CFDRFCC_RFIGCV_Pos) |                               \
   (CONFIG_RZV2H_CANFD_RXFIFO##n##_DEPTH                             \
      << R_CANFD_CFDRFCC_RFDC_Pos) |                                 \
   (CONFIG_RZV2H_CANFD_RXFIFO##n##_PAYLOAD                           \
      << R_CANFD_CFDRFCC_RFPLS_Pos) |                                \
   ((CONFIG_RZV2H_CANFD_RXFIFO##n##_INT_MODE >= 1)                   \
      ? R_CANFD_CFDRFCC_RFIE_Msk : 0) |                              \
   ((CONFIG_RZV2H_CANFD_RXFIFO##n##_INT_MODE >= 2)                   \
      ? R_CANFD_CFDRFCC_RFIM_Msk : 0) |                              \
   RZV2H_CANFD_RXFIFO_EN(n))

/* CFDCnCTR error-interrupt enable bits
 *
 * R_CANFD_Open() writes this value as is into the channel's CFDCnCTR
 * while the channel is in CH_RESET, the only mode in which these interrupt
 * enable bits can be written.
 *
 * CFDCnCTR.TAIE (Transmission Abort Interrupt Enable) is always set:
 * - CANIOC_OFLUSH and the bus-off recovery ioctls abort the busy TX
 *   message buffers. An aborted buffer gets CFDTMSTSn.TMTRF = 01b and
 *   stays busy (RZV2H_CANFD_TMSTS_BUSY) until CFDTMSTSn.TMTRF is cleared.
 * - The FSP TX ISR clears it, but an abort only raises the TX interrupt
 *   when CFDCnCTR.TAIE = 1b. Without it the buffer would stay busy until
 *   the next channel reset.
 *
 * Refer to "Channel n Control Register (CFDCnCTR)" in the HW manual.
 */

#define RZV2H_CANFD_ERRINT(ch, type, bit) \
    (CONFIG_RZV2H_CANFD##ch##_ERR_INT_##type ? \
     R_CANFD_CFDC_CTR_##bit##_Msk : 0u)

#define RZV2H_CANFD_ERROR_INTERRUPTS(ch) \
    (RZV2H_CANFD_ERRINT(ch, ERR_WARN, EWIE) | \
     RZV2H_CANFD_ERRINT(ch, ERR_PASSIVE, EPIE) | \
     RZV2H_CANFD_ERRINT(ch, BUS_OFF_ENTRY, BOEIE) | \
     RZV2H_CANFD_ERRINT(ch, BUS_OFF_RECOVERY, BORIE) | \
     RZV2H_CANFD_ERRINT(ch, OVERLOAD, OLIE) | \
     R_CANFD_CFDC_CTR_TAIE_Msk)

/* Common FIFO 0-17 configuration macro */

#define RZV2H_CANFD_CFG_COMMONFIFO(n)                                \
  (((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_ENABLE)                     \
      << R_CANFD_CFDCFCC_CFE_Pos) |                                  \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_RX_INT_ENABLE)              \
      << R_CANFD_CFDCFCC_CFRXIE_Pos) |                               \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_TX_INT_ENABLE)              \
      << R_CANFD_CFDCFCC_CFTXIE_Pos) |                               \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_PAYLOAD)                    \
      << R_CANFD_CFDCFCC_CFPLS_Pos) |                                \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_MODE)                       \
      << R_CANFD_CFDCFCC_CFM_Pos) |                                  \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_INTERVAL_TIMER)             \
      << R_CANFD_CFDCFCC_CFITSS_Pos) |                               \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_INTERVAL_TIMER_RES)         \
      << R_CANFD_CFDCFCC_CFITR_Pos) |                                \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_INT_MODE)                   \
      << R_CANFD_CFDCFCC_CFIM_Pos) |                                 \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_INT_THRESHOLD)              \
      << R_CANFD_CFDCFCC_CFIGCV_Pos) |                               \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_TXMB)                       \
      << R_CANFD_CFDCFCC_CFTML_Pos) |                                \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_DEPTH)                      \
      << R_CANFD_CFDCFCC_CFDC_Pos) |                                 \
   ((CONFIG_RZV2H_CANFD_COMMON_FIFO##n##_INTERVAL_TX_DELAY)          \
      << R_CANFD_CFDCFCC_CFITT_Pos))

/* At least one global error source is enabled, so GERR can fire */

#if defined(CONFIG_RZV2H_CANFD_GLOBAL_ERR_DEIE) || \
    defined(CONFIG_RZV2H_CANFD_GLOBAL_ERR_MEIE) || \
    defined(CONFIG_RZV2H_CANFD_GLOBAL_ERR_CMPOFIE)
#  define RZV2H_CANFD_HAVE_GLOBAL_ERR
#endif

/* At least one channel has a transceiver STB pin */

#if defined(CONFIG_RZV2H_CANFD0_STB_PIN) || \
    defined(CONFIG_RZV2H_CANFD1_STB_PIN) || \
    defined(CONFIG_RZV2H_CANFD2_STB_PIN) || \
    defined(CONFIG_RZV2H_CANFD3_STB_PIN) || \
    defined(CONFIG_RZV2H_CANFD4_STB_PIN) || \
    defined(CONFIG_RZV2H_CANFD5_STB_PIN)
#  define RZV2H_CANFD_HAVE_STB
#endif

/* Global error callback channel. The FSP reports every global error on
 * channel CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH, so this build check makes sure
 * that channel is enabled. Only checked when this build enables a global
 * error source.
 */

#ifdef RZV2H_CANFD_HAVE_GLOBAL_ERR
#  if CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH == 0 && !defined(CONFIG_RZV2H_CANFD0)
#    error "CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH = 0 needs CONFIG_RZV2H_CANFD0"
#  elif CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH == 1 && !defined(CONFIG_RZV2H_CANFD1)
#    error "CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH = 1 needs CONFIG_RZV2H_CANFD1"
#  elif CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH == 2 && !defined(CONFIG_RZV2H_CANFD2)
#    error "CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH = 2 needs CONFIG_RZV2H_CANFD2"
#  elif CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH == 3 && !defined(CONFIG_RZV2H_CANFD3)
#    error "CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH = 3 needs CONFIG_RZV2H_CANFD3"
#  elif CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH == 4 && !defined(CONFIG_RZV2H_CANFD4)
#    error "CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH = 4 needs CONFIG_RZV2H_CANFD4"
#  elif CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH == 5 && !defined(CONFIG_RZV2H_CANFD5)
#    error "CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH = 5 needs CONFIG_RZV2H_CANFD5"
#  endif
#endif

/* Options of the selected interface: the CAN character driver
 * (CONFIG_CAN_*) or SocketCAN (CONFIG_NET_CAN_*).
 */

#ifdef CONFIG_RZV2H_CANFD_SOCKET
#  ifdef CONFIG_NET_CAN_EXTID
#    define RZV2H_CANFD_HAVE_EXTID
#  endif
#  ifdef CONFIG_NET_CAN_CANFD
#    define RZV2H_CANFD_HAVE_FD
#  endif
#  ifdef CONFIG_NET_CAN_ERRORS
#    define RZV2H_CANFD_HAVE_ERRORS
#  endif
#  ifdef CONFIG_RZV2H_CANFD_SOCK_LOOPBACK
#    define RZV2H_CANFD_HAVE_LOOPBACK
#  endif
#else
#  ifdef CONFIG_CAN_EXTID
#    define RZV2H_CANFD_HAVE_EXTID
#  endif
#  ifdef CONFIG_CAN_FD
#    define RZV2H_CANFD_HAVE_FD
#  endif
#  ifdef CONFIG_CAN_ERRORS
#    define RZV2H_CANFD_HAVE_ERRORS
#  endif
#  ifdef CONFIG_CAN_LOOPBACK
#    define RZV2H_CANFD_HAVE_LOOPBACK
#  endif
#endif

/* Filter add/delete and bus-off recovery: always with the character
 * driver, with CONFIG_NETDEV_CAN_FILTER_IOCTL for SocketCAN. That option
 * also enables SIOCCANRECOVERY in net/netdev/netdev_ioctl.c.
 */

#if !defined(CONFIG_RZV2H_CANFD_SOCKET) || \
    defined(CONFIG_NETDEV_CAN_FILTER_IOCTL)
#  define RZV2H_CANFD_HAVE_FILTER_IOCTL
#endif

/* Restart of a running channel through CH_RESET: used by filters, bit
 * timing, CANIOC_SET_CONNMODES and loopback mode.
 */

#if defined(RZV2H_CANFD_HAVE_FILTER_IOCTL) || \
    defined(RZV2H_CANFD_HAVE_LOOPBACK)
#  define RZV2H_CANFD_HAVE_RESTART
#endif

/* Data phase sample point of the automatic bit timing. SocketCAN can set
 * it apart from the nominal one with SIOCSCANBITRATE.
 */

#ifdef CONFIG_RZV2H_CANFD_SOCKET
#  define RZV2H_CANFD_DATA_SP(priv)  ((priv)->sample_point_data)
#else
#  define RZV2H_CANFD_DATA_SP(priv)  ((priv)->sample_point)
#endif

/* Error report classes and detail bits. The CAN character driver
 * (CAN_ERROR*, include/nuttx/can/can.h) and SocketCAN (CAN_ERR_*,
 * include/nuttx/can.h) use the same values.
 */

#ifdef RZV2H_CANFD_HAVE_ERRORS
#  ifdef CONFIG_RZV2H_CANFD_SOCKET
#    define RZV2H_CANFD_ERR_DLC          CAN_ERR_DLC
#    define RZV2H_CANFD_ERR_CONTROLLER   CAN_ERR_CRTL
#    define RZV2H_CANFD_ERR_PROTOCOL     CAN_ERR_PROT
#    define RZV2H_CANFD_ERR_BUSOFF       CAN_ERR_BUSOFF
#    define RZV2H_CANFD_ERR_RESTARTED    CAN_ERR_RESTARTED
#    define RZV2H_CANFD_ERR1_RXOVERFLOW  CAN_ERR_CRTL_RX_OVERFLOW
#    define RZV2H_CANFD_ERR1_RXWARNING   CAN_ERR_CRTL_RX_WARNING
#    define RZV2H_CANFD_ERR1_TXWARNING   CAN_ERR_CRTL_TX_WARNING
#    define RZV2H_CANFD_ERR1_RXPASSIVE   CAN_ERR_CRTL_RX_PASSIVE
#    define RZV2H_CANFD_ERR1_TXPASSIVE   CAN_ERR_CRTL_TX_PASSIVE
#    define RZV2H_CANFD_ERR2_OVERLOAD    CAN_ERR_PROT_OVERLOAD
#  else
#    define RZV2H_CANFD_ERR_DLC          CAN_ERROR_DLC
#    define RZV2H_CANFD_ERR_CONTROLLER   CAN_ERROR_CONTROLLER
#    define RZV2H_CANFD_ERR_PROTOCOL     CAN_ERROR_PROTOCOL
#    define RZV2H_CANFD_ERR_BUSOFF       CAN_ERROR_BUSOFF
#    define RZV2H_CANFD_ERR_RESTARTED    CAN_ERROR_RESTARTED
#    define RZV2H_CANFD_ERR1_RXOVERFLOW  CAN_ERROR1_RXOVERFLOW
#    define RZV2H_CANFD_ERR1_RXWARNING   CAN_ERROR1_RXWARNING
#    define RZV2H_CANFD_ERR1_TXWARNING   CAN_ERROR1_TXWARNING
#    define RZV2H_CANFD_ERR1_RXPASSIVE   CAN_ERROR1_RXPASSIVE
#    define RZV2H_CANFD_ERR1_TXPASSIVE   CAN_ERROR1_TXPASSIVE
#    define RZV2H_CANFD_ERR2_OVERLOAD    CAN_ERROR2_OVERLOAD
#  endif
#endif

/* Send one error report to the upper half: can_receive() for the
 * character driver, the RX queue of the interface for SocketCAN.
 */

#ifdef CONFIG_RZV2H_CANFD_SOCKET
#  define RZV2H_CANFD_REPORT_ERROR(priv, hdr, data) \
     rzv2h_canfd_sock_error(priv, hdr, data)
#else
#  define RZV2H_CANFD_REPORT_ERROR(priv, hdr, data) \
     can_receive(&(priv)->dev, hdr, data)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CANFD pin configuration */

struct rzv2h_canfd_pin_config_s
{
  gpio_pinset_t tx_pin;                /* GPIO configuration for CAN TX */
  gpio_pinset_t rx_pin;                /* GPIO configuration for CAN RX */
#ifdef RZV2H_CANFD_HAVE_STB
  bool          stb;                   /* Transceiver has an STB pin */
  gpio_pinset_t stb_pin;               /* GPIO configuration for STB */
#endif
};

/* One filter to add, from CANIOC_ADD_STDFILTER, CANIOC_ADD_EXTFILTER,
 * SIOCACANSTDFILTER or SIOCACANEXTFILTER
 */

struct rzv2h_canfd_filter_s
{
  uint32_t id1;                        /* ID, lower ID of a range */
  uint32_t id2;                        /* Mask, second ID or upper ID */
  uint8_t  type;                       /* CAN_FILTER_* */
  uint8_t  prio;                       /* CAN_MSGPRIO_* */
  bool     ext;                        /* true: extended ID filter */
};

/* Per-channel AFL rule usage. A filter can take more than one AFL entry
 * (CAN_FILTER_DUAL and CAN_FILTER_RANGE). Its number is its first entry.
 */

struct rzv2h_canfd_afl_usage_s
{
  uint8_t  *owner;                     /* Per entry: 0 = free, else
                                        * RZV2H_AFL_OWNER() of its filter */
  uint32_t  num_rules;
};

/* Error report header. The character driver uses its own type. SocketCAN
 * uses a type with the same field names: the error reports below set only
 * these fields.
 */

#ifdef CONFIG_RZV2H_CANFD_SOCKET
struct rzv2h_canfd_errhdr_s
{
  uint32_t ch_id;                      /* Error class (CAN_ERR_*) */
  uint8_t  ch_dlc;                     /* CAN_ERR_DLC */
  uint8_t  ch_error;                   /* Set to 1 */
};

typedef struct rzv2h_canfd_errhdr_s    rzv2h_canfd_errhdr_t;

/* One received frame or error report in the SocketCAN RX queue. The
 * first 16 bytes of a struct canfd_frame are a struct can_frame.
 */

struct rzv2h_canfd_sock_rx_s
{
  struct canfd_frame frame;
  bool               fd;               /* true: CAN FD frame */
};
#else
typedef struct can_hdr_s               rzv2h_canfd_errhdr_t;
#endif

/* Per-channel private data structure */

struct rzv2h_canfd_priv_s
{
  /* Bit timing configuration */

  uint32_t baud;                       /* Configured nominal baud rate */
  uint32_t baud_data;                  /* Configured data baud rate
                                        * (CAN FD) */
  uint32_t sample_point;               /* Desired sample point (60-99%) */
  bool     use_manual_timing;          /* Use manual bit timing from
                                        * Kconfig */

  /* Run state */

  bool     started;                    /* Channel started flag */

  /* Pin configuration */

  struct rzv2h_canfd_pin_config_s *pin_config;

  /* Register base pointer (computed at setup time from channel) */

  volatile R_CANFD_Type *p_reg;

  /* FSP HAL instance pointer — holds p_ctrl, p_cfg */

  can_instance_t const *hal_inst;

  /* FSP global configuration, shared by all channels */

  canfd_global_cfg_t   *p_fsp_global;

  /* AFL rule usage */

  struct rzv2h_canfd_afl_usage_s *afl_usage;

  /* RX FIFO type: which RX FIFO(s) this channel receives in */

  uint8_t  rx_fifo_type;               /* RZV2H_CANFD_RX_FIFO_TYPE_* */

  /* TX message buffers used */

  uint8_t  txmb_num;

  /* One bit per TX message buffer, set by CANIOC_OFLUSH while the buffer
   * is still busy. The upper half has already dropped the frame in that
   * buffer, so the next "sent" or "aborted" report of the FSP TX ISR for
   * it is ignored instead of calling can_txdone().
   */

  uint16_t tx_discard;

  /* Selectable interrupt event IDs (INTSEL peripheral event sources) */

  rzv2h_irqsel_t err_irq_event;         /* ERR INTSEL event ID */
  rzv2h_irqsel_t rx_irq_event;          /* REC INTSEL event ID */
  rzv2h_irqsel_t tx_irq_event;          /* TRX INTSEL event ID */

#ifdef CONFIG_RZV2H_CANFD_SOCKET
  /* Data phase sample point, set with SIOCSCANBITRATE */

  uint32_t sample_point_data;

  /* RX queue: filled in interrupt context, emptied by
   * rzv2h_canfd_sock_receive(). rx_lost is set when a frame is dropped
   * because the queue is full.
   */

  struct rzv2h_canfd_sock_rx_s rx_queue[CONFIG_RZV2H_CANFD_SOCK_RX_DEPTH];
  uint8_t  rx_head;                    /* Oldest entry */
  uint8_t  rx_count;                   /* Number of entries */
  bool     rx_lost;                    /* A frame was dropped */

  /* Packet of the frame loaded into each TX message buffer, held until
   * rzv2h_canfd_sock_reclaim() frees it, so the TX quota counts the frames
   * still in the hardware.
   */

  netpkt_t *tx_pkt[RZV2H_CANFD_SOCK_TXMB_MAX];

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  /* TX deadline of the frame in each TX message buffer. 0 means no
   * deadline. Cleared when the frame is sent or aborted. tx_wdog fires at
   * the earliest deadline still set.
   */

  struct timespec tx_deadline[RZV2H_CANFD_SOCK_TXMB_MAX];
  struct wdog_s tx_wdog;
#endif

  /* NuttX network device */

  struct netdev_lowerhalf_s dev;
#else
  /* NuttX CAN device structure — MUST be last member */

  struct can_dev_s dev;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Channel start, stop and interrupts, used by both interfaces */

static int  rzv2h_canfd_chan_setup(struct rzv2h_canfd_priv_s *priv);
static void rzv2h_canfd_chan_shutdown(struct rzv2h_canfd_priv_s *priv);
static void rzv2h_canfd_chan_rxint(struct rzv2h_canfd_priv_s *priv,
                                   bool enable);
static void rzv2h_canfd_chan_txint(struct rzv2h_canfd_priv_s *priv,
                                   bool enable);

#ifdef RZV2H_CANFD_HAVE_RESTART
static int  rzv2h_canfd_chan_reset(struct rzv2h_canfd_priv_s *priv,
                                   can_test_mode_t *test_mode);
static int  rzv2h_canfd_chan_run(struct rzv2h_canfd_priv_s *priv,
                                 can_test_mode_t test_mode);
#endif

#ifdef CONFIG_RZV2H_CANFD_SOCKET
/* netdev_ops_s implementation */

static int  rzv2h_canfd_sock_ifup(struct netdev_lowerhalf_s *dev);
static int  rzv2h_canfd_sock_ifdown(struct netdev_lowerhalf_s *dev);
static int  rzv2h_canfd_sock_transmit(struct netdev_lowerhalf_s *dev,
                                      netpkt_t *pkt);
static netpkt_t *rzv2h_canfd_sock_receive(struct netdev_lowerhalf_s *dev);
static void rzv2h_canfd_sock_reclaim(struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NETDEV_IOCTL
static int  rzv2h_canfd_sock_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                                   unsigned long arg);
#endif

/* SocketCAN RX queue and held TX packets */

static void rzv2h_canfd_sock_txfree(struct rzv2h_canfd_priv_s *priv);

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
static unsigned int rzv2h_canfd_sock_deadline_get(
                        struct netdev_lowerhalf_s *dev, netpkt_t *pkt,
                        unsigned int len, struct timespec *deadline);
static void rzv2h_canfd_sock_deadline_arm(struct rzv2h_canfd_priv_s *priv);
static void rzv2h_canfd_sock_deadline_clear(struct rzv2h_canfd_priv_s *priv,
                                            uint32_t mb);
static void rzv2h_canfd_sock_deadline_expiry(wdparm_t arg);
#endif

static void rzv2h_canfd_sock_rxpush(struct rzv2h_canfd_priv_s *priv,
                                    const struct canfd_frame *frame,
                                    bool fd);

#ifdef RZV2H_CANFD_HAVE_ERRORS
static void rzv2h_canfd_sock_error(struct rzv2h_canfd_priv_s *priv,
                                   const rzv2h_canfd_errhdr_t *hdr,
                                   const uint8_t *data);
#endif
#else
/* can_ops_s implementation */

static void rzv2h_canfd_reset(struct can_dev_s *dev);
static int  rzv2h_canfd_setup(struct can_dev_s *dev);
static void rzv2h_canfd_shutdown(struct can_dev_s *dev);
static void rzv2h_canfd_rxint(struct can_dev_s *dev, bool enable);
static void rzv2h_canfd_txint(struct can_dev_s *dev, bool enable);
static int  rzv2h_canfd_ioctl(struct can_dev_s *dev, int cmd,
                              unsigned long arg);
static int  rzv2h_canfd_send(struct can_dev_s *dev,
                             struct can_msg_s *msg);
static bool rzv2h_canfd_txready(struct can_dev_s *dev);
static bool rzv2h_canfd_txempty(struct can_dev_s *dev);
#endif

/* Internal helpers */

static int  rzv2h_canfd_baud_to_timing(uint32_t baud,
                                       uint32_t sample_point,
                                       bool is_data,
                                       bool delay_compensation,
                                       can_bit_timing_cfg_t *timing);
static int  rzv2h_canfd_timing_check(const can_bit_timing_cfg_t *t,
                                     bool is_data);
static uint32_t rzv2h_canfd_bittiming_clocks(const can_bit_timing_cfg_t *t);
static void rzv2h_canfd_set_filter_accept_all(canfd_afl_entry_t *entry,
                                              canfd_rx_fifo_t fifo_flags);
static void rzv2h_canfd_set_filter_disable(canfd_afl_entry_t *entry);
static canfd_rx_fifo_t rzv2h_canfd_rx_fifo_flags(
                         struct rzv2h_canfd_priv_s *priv, uint8_t prio);
static struct rzv2h_canfd_priv_s *rzv2h_canfd_ch_priv(uint32_t channel);
static int  rzv2h_canfd_rx_fifo_check_cfg(struct rzv2h_canfd_priv_s *priv);
static void rzv2h_canfd_rx_flush(struct rzv2h_canfd_priv_s *priv);
static void rzv2h_canfd_rx_deliver(struct rzv2h_canfd_priv_s *priv,
                                   const can_frame_t *frame);
static void rzv2h_canfd_cfifo_clear_lost(uint32_t cfifo);
static void rzv2h_canfd_rxfifo_clear_lost(uint32_t rxfifo);

#ifdef RZV2H_CANFD_HAVE_ERRORS
static void rzv2h_canfd_report_rx_overflow(struct rzv2h_canfd_priv_s *priv);
static void rzv2h_canfd_channel_error(struct rzv2h_canfd_priv_s *priv,
                                      uint32_t error);
#endif

static void rzv2h_canfd_global_error(void);
static void rzv2h_canfd_cfifo_rx_interrupt(struct rzv2h_canfd_priv_s *priv);
static bool rzv2h_canfd_txmb_busy(struct rzv2h_canfd_priv_s *priv,
                                  uint32_t mb);
static int  rzv2h_canfd_txmb_next(struct rzv2h_canfd_priv_s *priv);

#ifdef RZV2H_CANFD_HAVE_FILTER_IOCTL
static uint32_t rzv2h_canfd_afl_count_used(struct rzv2h_canfd_priv_s *priv);
static int32_t rzv2h_canfd_afl_free_entry(struct rzv2h_canfd_priv_s *priv,
                                          uint32_t start);
static void rzv2h_canfd_afl_write_entry(struct rzv2h_canfd_priv_s *priv,
                                        uint32_t afl_entry,
                                        const canfd_afl_entry_t *entry);
static void rzv2h_canfd_set_filter_rule(canfd_afl_entry_t *entry, bool ext,
                                        uint32_t id, uint32_t mask,
                                        canfd_rx_fifo_t fifo_flags);
static int  rzv2h_canfd_afl_add(struct rzv2h_canfd_priv_s *priv,
                                const struct rzv2h_canfd_filter_s *req);
static int  rzv2h_canfd_afl_del(struct rzv2h_canfd_priv_s *priv,
                                uint32_t num, bool ext);
#endif

static int  rzv2h_canfd_irq_attach(struct rzv2h_canfd_priv_s *priv);
static void rzv2h_canfd_irq_detach(struct rzv2h_canfd_priv_s *priv);
static int  rzv2h_canfd_global_irq_attach(void);
static void rzv2h_canfd_global_irq_detach(void);

/* ioctl command handlers */

#ifndef CONFIG_RZV2H_CANFD_SOCKET
static int  rzv2h_canfd_ioctl_set_bittiming(struct rzv2h_canfd_priv_s *priv,
                                            unsigned long arg);
static int  rzv2h_canfd_ioctl_get_bittiming(struct rzv2h_canfd_priv_s *priv,
                                            unsigned long arg);
static int  rzv2h_canfd_ioctl_set_connmodes(struct rzv2h_canfd_priv_s *priv,
                                            unsigned long arg);
static int  rzv2h_canfd_ioctl_get_connmodes(struct rzv2h_canfd_priv_s *priv,
                                            unsigned long arg);
static int  rzv2h_canfd_ioctl_iflush(struct rzv2h_canfd_priv_s *priv,
                                     unsigned long arg);
static int  rzv2h_canfd_ioctl_oflush(struct rzv2h_canfd_priv_s *priv,
                                     unsigned long arg);
static int  rzv2h_canfd_ioctl_ioflush(struct rzv2h_canfd_priv_s *priv,
                                      unsigned long arg);
#endif

#ifndef CONFIG_RZV2H_CANFD_SOCKET
static int  rzv2h_canfd_ioctl_add_stdfilter(struct rzv2h_canfd_priv_s *priv,
                                            unsigned long arg);
static int  rzv2h_canfd_ioctl_del_stdfilter(struct rzv2h_canfd_priv_s *priv,
                                            unsigned long arg);
#ifdef CONFIG_CAN_EXTID
static int  rzv2h_canfd_ioctl_add_extfilter(struct rzv2h_canfd_priv_s *priv,
                                            unsigned long arg);
static int  rzv2h_canfd_ioctl_del_extfilter(struct rzv2h_canfd_priv_s *priv,
                                            unsigned long arg);
#endif
#endif

#ifdef RZV2H_CANFD_HAVE_FILTER_IOCTL
static int rzv2h_canfd_ioctl_busoff_recovery(struct rzv2h_canfd_priv_s *priv,
                                             unsigned long arg);
#endif

/* Forward declarations for FSP callback */

static void rzv2h_canfd_fsp_callback(can_callback_args_t *p_args);

/* FSP ISR entry points */

void canfd_error_isr(void);
void canfd_rx_fifo_isr(void);
void canfd_channel_tx_isr(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RZV2H_CANFD_SOCKET
/* SocketCAN network device operations */

static const struct netdev_ops_s g_rzv2h_canfd_sock_ops =
{
  .ifup     = rzv2h_canfd_sock_ifup,
  .ifdown   = rzv2h_canfd_sock_ifdown,
  .transmit = rzv2h_canfd_sock_transmit,
  .receive  = rzv2h_canfd_sock_receive,
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = rzv2h_canfd_sock_ioctl,
#endif
  .reclaim  = rzv2h_canfd_sock_reclaim,
};
#else
/* CANFD operations vtable */

static const struct can_ops_s g_rzv2h_canfd_ops =
{
  .co_reset         = rzv2h_canfd_reset,
  .co_setup         = rzv2h_canfd_setup,
  .co_shutdown      = rzv2h_canfd_shutdown,
  .co_rxint         = rzv2h_canfd_rxint,
  .co_txint         = rzv2h_canfd_txint,
  .co_ioctl         = rzv2h_canfd_ioctl,
  .co_remoterequest = NULL,            /* Use co_send for RTR */
  .co_send          = rzv2h_canfd_send,
  .co_txready       = rzv2h_canfd_txready,
  .co_txempty       = rzv2h_canfd_txempty,
  .co_cancel        = NULL,
};
#endif

/* Global configuration (shared across all channels) */

static canfd_global_cfg_t g_canfd_global_cfg =
{
  .global_interrupts =
#ifdef CONFIG_RZV2H_CANFD_GLOBAL_ERR_DEIE
    R_CANFD_CFDGCTR_DEIE_Msk |
#endif
#ifdef CONFIG_RZV2H_CANFD_GLOBAL_ERR_MEIE
    R_CANFD_CFDGCTR_MEIE_Msk |
#endif
#ifdef CONFIG_RZV2H_CANFD_GLOBAL_ERR_CMPOFIE
    R_CANFD_CFDGCTR_CMPOFIE_Msk |
#endif
    RZV2H_CANFD_GMDC_NO_CHANGE,

  /* DCS = 0 (CANFD_0_clkc); DCS = 1 is prohibited on RZ/V2H.
   * CMPOC = 0b rejects a message larger than the FIFO payload size
   * ("Payload Overflow Reject"), 1b cuts its payload to fit.
   */

  .global_config     =
    (CONFIG_RZV2H_CANFD_TRANSMIT_PRIORITY << R_CANFD_CFDGCFG_TPRI_Pos) |
    (CONFIG_RZV2H_CANFD_DLC_CHECK << R_CANFD_CFDGCFG_DCE_Pos) |
#ifndef CONFIG_RZV2H_CANFD_FD_OVERFLOW
    R_CANFD_CFDGCFG_CMPOC_Msk |
#endif
    ((uint32_t)CONFIG_RZV2H_CANFD_TIMER_PRESCALER <<
     R_CANFD_CFDGCFG_ITRCP_Pos),

  .rx_mb_config      = (CONFIG_RZV2H_CANFD_RXMB_NUMBER |
                        (CONFIG_RZV2H_CANFD_RXMB_SIZE << 16)),
  .global_err_ipl    = CONFIG_RZV2H_CANFD_GLOBAL_ERR_PRIORITY,
  .rx_fifo_ipl       = CONFIG_RZV2H_CANFD_RX_FIFO_PRIORITY,
  .rx_fifo_config    =
  {
    RZV2H_CANFD_RXFIFO_CFG(0),
    RZV2H_CANFD_RXFIFO_CFG(1),
    RZV2H_CANFD_RXFIFO_CFG(2),
    RZV2H_CANFD_RXFIFO_CFG(3),
    RZV2H_CANFD_RXFIFO_CFG(4),
    RZV2H_CANFD_RXFIFO_CFG(5),
    RZV2H_CANFD_RXFIFO_CFG(6),
    RZV2H_CANFD_RXFIFO_CFG(7),
  },
  .common_fifo_config =
  {
    RZV2H_CANFD_CFG_COMMONFIFO(0),
    RZV2H_CANFD_CFG_COMMONFIFO(1),
    RZV2H_CANFD_CFG_COMMONFIFO(2),
    RZV2H_CANFD_CFG_COMMONFIFO(3),
    RZV2H_CANFD_CFG_COMMONFIFO(4),
    RZV2H_CANFD_CFG_COMMONFIFO(5),
    RZV2H_CANFD_CFG_COMMONFIFO(6),
    RZV2H_CANFD_CFG_COMMONFIFO(7),
    RZV2H_CANFD_CFG_COMMONFIFO(8),
    RZV2H_CANFD_CFG_COMMONFIFO(9),
    RZV2H_CANFD_CFG_COMMONFIFO(10),
    RZV2H_CANFD_CFG_COMMONFIFO(11),
    RZV2H_CANFD_CFG_COMMONFIFO(12),
    RZV2H_CANFD_CFG_COMMONFIFO(13),
    RZV2H_CANFD_CFG_COMMONFIFO(14),
    RZV2H_CANFD_CFG_COMMONFIFO(15),
    RZV2H_CANFD_CFG_COMMONFIFO(16),
    RZV2H_CANFD_CFG_COMMONFIFO(17),
  },
};

/* Number of open channels. The global error and RX FIFO IRQs are shared,
 * so the first rzv2h_canfd_setup() attaches them and the last
 * rzv2h_canfd_shutdown() detaches them.
 */

static int g_canfd_global_irq_refs = 0;

/* Channels with RX on, bit n = channel n */

static uint32_t g_canfd_rxint_mask = 0;

#ifdef CONFIG_RZV2H_CANFD_SOCKET
/* Serializes channel start and stop: rzv2h_canfd_sock_ifup() and
 * rzv2h_canfd_sock_ifdown() of different interfaces can run at the same
 * time, and they share the state above and the FSP global setup.
 */

static mutex_t g_canfd_lock = NXMUTEX_INITIALIZER;
#endif

/* Per-channel static instances */

#ifdef CONFIG_RZV2H_CANFD0
static struct rzv2h_canfd_priv_s g_canfd0_priv;
static canfd_afl_entry_t g_canfd0_afl_table[CANFD_CFG_AFL_CH0_RULE_NUM];
static uint8_t g_canfd0_afl_owner[CANFD_CFG_AFL_CH0_RULE_NUM];
static struct rzv2h_canfd_afl_usage_s g_canfd0_afl_usage =
{
  .owner     = g_canfd0_afl_owner,
  .num_rules = CANFD_CFG_AFL_CH0_RULE_NUM,
};
#endif
#ifdef CONFIG_RZV2H_CANFD1
static struct rzv2h_canfd_priv_s g_canfd1_priv;
static canfd_afl_entry_t g_canfd1_afl_table[CANFD_CFG_AFL_CH1_RULE_NUM];
static uint8_t g_canfd1_afl_owner[CANFD_CFG_AFL_CH1_RULE_NUM];
static struct rzv2h_canfd_afl_usage_s g_canfd1_afl_usage =
{
  .owner     = g_canfd1_afl_owner,
  .num_rules = CANFD_CFG_AFL_CH1_RULE_NUM,
};
#endif
#ifdef CONFIG_RZV2H_CANFD2
static struct rzv2h_canfd_priv_s g_canfd2_priv;
static canfd_afl_entry_t g_canfd2_afl_table[CANFD_CFG_AFL_CH2_RULE_NUM];
static uint8_t g_canfd2_afl_owner[CANFD_CFG_AFL_CH2_RULE_NUM];
static struct rzv2h_canfd_afl_usage_s g_canfd2_afl_usage =
{
  .owner     = g_canfd2_afl_owner,
  .num_rules = CANFD_CFG_AFL_CH2_RULE_NUM,
};
#endif
#ifdef CONFIG_RZV2H_CANFD3
static struct rzv2h_canfd_priv_s g_canfd3_priv;
static canfd_afl_entry_t g_canfd3_afl_table[CANFD_CFG_AFL_CH3_RULE_NUM];
static uint8_t g_canfd3_afl_owner[CANFD_CFG_AFL_CH3_RULE_NUM];
static struct rzv2h_canfd_afl_usage_s g_canfd3_afl_usage =
{
  .owner     = g_canfd3_afl_owner,
  .num_rules = CANFD_CFG_AFL_CH3_RULE_NUM,
};
#endif
#ifdef CONFIG_RZV2H_CANFD4
static struct rzv2h_canfd_priv_s g_canfd4_priv;
static canfd_afl_entry_t g_canfd4_afl_table[CANFD_CFG_AFL_CH4_RULE_NUM];
static uint8_t g_canfd4_afl_owner[CANFD_CFG_AFL_CH4_RULE_NUM];
static struct rzv2h_canfd_afl_usage_s g_canfd4_afl_usage =
{
  .owner     = g_canfd4_afl_owner,
  .num_rules = CANFD_CFG_AFL_CH4_RULE_NUM,
};
#endif
#ifdef CONFIG_RZV2H_CANFD5
static struct rzv2h_canfd_priv_s g_canfd5_priv;
static canfd_afl_entry_t g_canfd5_afl_table[CANFD_CFG_AFL_CH5_RULE_NUM];
static uint8_t g_canfd5_afl_owner[CANFD_CFG_AFL_CH5_RULE_NUM];
static struct rzv2h_canfd_afl_usage_s g_canfd5_afl_usage =
{
  .owner     = g_canfd5_afl_owner,
  .num_rules = CANFD_CFG_AFL_CH5_RULE_NUM,
};
#endif

/* Per-channel static FSP instances. */

#ifdef CONFIG_RZV2H_CANFD0

static struct rzv2h_canfd_pin_config_s g_rzv2h_canfd0_pin_config =
{
  .tx_pin = BOARD_CANFD0_TX_GPIO,
  .rx_pin = BOARD_CANFD0_RX_GPIO,
#ifdef CONFIG_RZV2H_CANFD0_STB_PIN
  .stb     = true,
  .stb_pin = BOARD_CANFD0_STB_GPIO,
#endif
};

static canfd_instance_ctrl_t g_canfd0_ctrl;

static can_bit_timing_cfg_t g_canfd0_timing =
{
#ifdef CONFIG_RZV2H_CANFD0_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD0_MANUAL_NORM_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD0_MANUAL_NORM_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD0_MANUAL_NORM_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD0_MANUAL_NORM_SYNC_JUMP,
#endif
};

static can_bit_timing_cfg_t g_canfd0_timing_data =
{
#ifdef CONFIG_RZV2H_CANFD0_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD0_MANUAL_DATA_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD0_MANUAL_DATA_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD0_MANUAL_DATA_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD0_MANUAL_DATA_SYNC_JUMP,
#endif
};

static canfd_extended_cfg_t g_canfd0_extend_cfg =
{
  .p_afl                 = g_canfd0_afl_table,
  .txmb_txi_enable       =
    RZV2H_CANFD_TXMB_MASK(CONFIG_RZV2H_CANFD0_TXMB_NUM),
  .error_interrupts      = RZV2H_CANFD_ERROR_INTERRUPTS(0),
  .p_data_timing         = &g_canfd0_timing_data,
#ifdef CONFIG_RZV2H_CANFD0_DELAY_COMPENSATION
  .delay_compensation    = 1,
#else
  .delay_compensation    = 0,
#endif
  .p_global_cfg          = &g_canfd_global_cfg,
  .txmb_merge_mode_enable = CANFD_TXMB_MERGE_MODE_DISABLE,
  .p_reg                 = (void *)R_CANFD,
};

static can_cfg_t g_canfd0_cfg =
{
  .channel      = 0,
  .p_bit_timing = &g_canfd0_timing,
  .p_callback   = rzv2h_canfd_fsp_callback,
  .p_context    = &g_canfd0_priv,
  .p_extend     = &g_canfd0_extend_cfg,
  .ipl          = CONFIG_RZV2H_CANFD0_INT_PRIORITY,
  .error_irq    = (IRQn_Type)CONFIG_RZV2H_CANFD0_ERR_INTSEL,
  .rx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD0_REC_INTSEL,
  .tx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD0_TRX_INTSEL,
};

static can_instance_t g_canfd0_hal_inst =
{
  .p_ctrl = (can_ctrl_t *)&g_canfd0_ctrl,
  .p_cfg  = &g_canfd0_cfg,
};

static struct rzv2h_canfd_priv_s g_canfd0_priv =
{
  .baud         = CONFIG_RZV2H_CANFD0_AUTO_NORM_RATE,
  .baud_data    = CONFIG_RZV2H_CANFD0_AUTO_DATA_RATE,
  .sample_point      = CONFIG_RZV2H_CANFD0_AUTO_SAMPLE_POINT,
#ifdef CONFIG_RZV2H_CANFD0_USE_MANUAL_BAUDRATE
  .use_manual_timing = true,
#else
  .use_manual_timing = false,
#endif
  .started           = false,
  .pin_config   = &g_rzv2h_canfd0_pin_config,
  .p_reg        = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE,
  .hal_inst     = &g_canfd0_hal_inst,
  .err_irq_event = RZV2H_IRQSEL_CANFD_ERR0,
  .rx_irq_event  = RZV2H_IRQSEL_CANFD_REC0,
  .tx_irq_event  = RZV2H_IRQSEL_CANFD_TRX0,
  .p_fsp_global = &g_canfd_global_cfg,
  .afl_usage    = &g_canfd0_afl_usage,
  .rx_fifo_type = CONFIG_RZV2H_CANFD0_RX_FIFO_TYPE,
  .txmb_num     = CONFIG_RZV2H_CANFD0_TXMB_NUM,
#ifndef CONFIG_RZV2H_CANFD_SOCKET
  .dev          =
  {
    .cd_ops  = &g_rzv2h_canfd_ops,
    .cd_priv = &g_canfd0_priv,
  },
#endif
};
#endif

#ifdef CONFIG_RZV2H_CANFD1

static struct rzv2h_canfd_pin_config_s g_rzv2h_canfd1_pin_config =
{
  .tx_pin = BOARD_CANFD1_TX_GPIO,
  .rx_pin = BOARD_CANFD1_RX_GPIO,
#ifdef CONFIG_RZV2H_CANFD1_STB_PIN
  .stb     = true,
  .stb_pin = BOARD_CANFD1_STB_GPIO,
#endif
};

static canfd_instance_ctrl_t g_canfd1_ctrl;

static can_bit_timing_cfg_t g_canfd1_timing =
{
#ifdef CONFIG_RZV2H_CANFD1_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD1_MANUAL_NORM_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD1_MANUAL_NORM_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD1_MANUAL_NORM_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD1_MANUAL_NORM_SYNC_JUMP,
#endif
};

static can_bit_timing_cfg_t g_canfd1_timing_data =
{
#ifdef CONFIG_RZV2H_CANFD1_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD1_MANUAL_DATA_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD1_MANUAL_DATA_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD1_MANUAL_DATA_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD1_MANUAL_DATA_SYNC_JUMP,
#endif
};

static canfd_extended_cfg_t g_canfd1_extend_cfg =
{
  .p_afl                 = g_canfd1_afl_table,
  .txmb_txi_enable       =
    RZV2H_CANFD_TXMB_MASK(CONFIG_RZV2H_CANFD1_TXMB_NUM),
  .error_interrupts      = RZV2H_CANFD_ERROR_INTERRUPTS(1),
  .p_data_timing         = &g_canfd1_timing_data,
#ifdef CONFIG_RZV2H_CANFD1_DELAY_COMPENSATION
  .delay_compensation    = 1,
#else
  .delay_compensation    = 0,
#endif
  .p_global_cfg          = &g_canfd_global_cfg,
  .txmb_merge_mode_enable = CANFD_TXMB_MERGE_MODE_DISABLE,
  .p_reg                 = (void *)R_CANFD,
};

static can_cfg_t g_canfd1_cfg =
{
  .channel      = 1,
  .p_bit_timing = &g_canfd1_timing,
  .p_callback   = rzv2h_canfd_fsp_callback,
  .p_context    = &g_canfd1_priv,
  .p_extend     = &g_canfd1_extend_cfg,
  .ipl          = CONFIG_RZV2H_CANFD1_INT_PRIORITY,
  .error_irq    = (IRQn_Type)CONFIG_RZV2H_CANFD1_ERR_INTSEL,
  .rx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD1_REC_INTSEL,
  .tx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD1_TRX_INTSEL,
};

static can_instance_t g_canfd1_hal_inst =
{
  .p_ctrl = (can_ctrl_t *)&g_canfd1_ctrl,
  .p_cfg  = &g_canfd1_cfg,
};

static struct rzv2h_canfd_priv_s g_canfd1_priv =
{
  .baud         = CONFIG_RZV2H_CANFD1_AUTO_NORM_RATE,
  .baud_data    = CONFIG_RZV2H_CANFD1_AUTO_DATA_RATE,
  .sample_point      = CONFIG_RZV2H_CANFD1_AUTO_SAMPLE_POINT,
#ifdef CONFIG_RZV2H_CANFD1_USE_MANUAL_BAUDRATE
  .use_manual_timing = true,
#else
  .use_manual_timing = false,
#endif
  .started           = false,
  .pin_config   = &g_rzv2h_canfd1_pin_config,
  .p_reg        = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE,
  .hal_inst     = &g_canfd1_hal_inst,
  .err_irq_event = RZV2H_IRQSEL_CANFD_ERR1,
  .rx_irq_event  = RZV2H_IRQSEL_CANFD_REC1,
  .tx_irq_event  = RZV2H_IRQSEL_CANFD_TRX1,
  .p_fsp_global = &g_canfd_global_cfg,
  .afl_usage    = &g_canfd1_afl_usage,
  .rx_fifo_type = CONFIG_RZV2H_CANFD1_RX_FIFO_TYPE,
  .txmb_num     = CONFIG_RZV2H_CANFD1_TXMB_NUM,
#ifndef CONFIG_RZV2H_CANFD_SOCKET
  .dev          =
  {
    .cd_ops  = &g_rzv2h_canfd_ops,
    .cd_priv = &g_canfd1_priv,
  },
#endif
};
#endif

#ifdef CONFIG_RZV2H_CANFD2

static struct rzv2h_canfd_pin_config_s g_rzv2h_canfd2_pin_config =
{
  .tx_pin = BOARD_CANFD2_TX_GPIO,
  .rx_pin = BOARD_CANFD2_RX_GPIO,
#ifdef CONFIG_RZV2H_CANFD2_STB_PIN
  .stb     = true,
  .stb_pin = BOARD_CANFD2_STB_GPIO,
#endif
};

static canfd_instance_ctrl_t g_canfd2_ctrl;

static can_bit_timing_cfg_t g_canfd2_timing =
{
#ifdef CONFIG_RZV2H_CANFD2_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD2_MANUAL_NORM_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD2_MANUAL_NORM_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD2_MANUAL_NORM_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD2_MANUAL_NORM_SYNC_JUMP,
#endif
};

static can_bit_timing_cfg_t g_canfd2_timing_data =
{
#ifdef CONFIG_RZV2H_CANFD2_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD2_MANUAL_DATA_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD2_MANUAL_DATA_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD2_MANUAL_DATA_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD2_MANUAL_DATA_SYNC_JUMP,
#endif
};

static canfd_extended_cfg_t g_canfd2_extend_cfg =
{
  .p_afl                 = g_canfd2_afl_table,
  .txmb_txi_enable       =
    RZV2H_CANFD_TXMB_MASK(CONFIG_RZV2H_CANFD2_TXMB_NUM),
  .error_interrupts      = RZV2H_CANFD_ERROR_INTERRUPTS(2),
  .p_data_timing         = &g_canfd2_timing_data,
#ifdef CONFIG_RZV2H_CANFD2_DELAY_COMPENSATION
  .delay_compensation    = 1,
#else
  .delay_compensation    = 0,
#endif
  .p_global_cfg          = &g_canfd_global_cfg,
  .txmb_merge_mode_enable = CANFD_TXMB_MERGE_MODE_DISABLE,
  .p_reg                 = (void *)R_CANFD,
};

static can_cfg_t g_canfd2_cfg =
{
  .channel      = 2,
  .p_bit_timing = &g_canfd2_timing,
  .p_callback   = rzv2h_canfd_fsp_callback,
  .p_context    = &g_canfd2_priv,
  .p_extend     = &g_canfd2_extend_cfg,
  .ipl          = CONFIG_RZV2H_CANFD2_INT_PRIORITY,
  .error_irq    = (IRQn_Type)CONFIG_RZV2H_CANFD2_ERR_INTSEL,
  .rx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD2_REC_INTSEL,
  .tx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD2_TRX_INTSEL,
};

static can_instance_t g_canfd2_hal_inst =
{
  .p_ctrl = (can_ctrl_t *)&g_canfd2_ctrl,
  .p_cfg  = &g_canfd2_cfg,
};

static struct rzv2h_canfd_priv_s g_canfd2_priv =
{
  .baud         = CONFIG_RZV2H_CANFD2_AUTO_NORM_RATE,
  .baud_data    = CONFIG_RZV2H_CANFD2_AUTO_DATA_RATE,
  .sample_point      = CONFIG_RZV2H_CANFD2_AUTO_SAMPLE_POINT,
#ifdef CONFIG_RZV2H_CANFD2_USE_MANUAL_BAUDRATE
  .use_manual_timing = true,
#else
  .use_manual_timing = false,
#endif
  .started           = false,
  .pin_config   = &g_rzv2h_canfd2_pin_config,
  .p_reg        = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE,
  .hal_inst     = &g_canfd2_hal_inst,
  .err_irq_event = RZV2H_IRQSEL_CANFD_ERR2,
  .rx_irq_event  = RZV2H_IRQSEL_CANFD_REC2,
  .tx_irq_event  = RZV2H_IRQSEL_CANFD_TRX2,
  .p_fsp_global = &g_canfd_global_cfg,
  .afl_usage    = &g_canfd2_afl_usage,
  .rx_fifo_type = CONFIG_RZV2H_CANFD2_RX_FIFO_TYPE,
  .txmb_num     = CONFIG_RZV2H_CANFD2_TXMB_NUM,
#ifndef CONFIG_RZV2H_CANFD_SOCKET
  .dev          =
  {
    .cd_ops  = &g_rzv2h_canfd_ops,
    .cd_priv = &g_canfd2_priv,
  },
#endif
};
#endif

#ifdef CONFIG_RZV2H_CANFD3

static struct rzv2h_canfd_pin_config_s g_rzv2h_canfd3_pin_config =
{
  .tx_pin = BOARD_CANFD3_TX_GPIO,
  .rx_pin = BOARD_CANFD3_RX_GPIO,
#ifdef CONFIG_RZV2H_CANFD3_STB_PIN
  .stb     = true,
  .stb_pin = BOARD_CANFD3_STB_GPIO,
#endif
};

static canfd_instance_ctrl_t g_canfd3_ctrl;

static can_bit_timing_cfg_t g_canfd3_timing =
{
#ifdef CONFIG_RZV2H_CANFD3_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD3_MANUAL_NORM_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD3_MANUAL_NORM_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD3_MANUAL_NORM_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD3_MANUAL_NORM_SYNC_JUMP,
#endif
};

static can_bit_timing_cfg_t g_canfd3_timing_data =
{
#ifdef CONFIG_RZV2H_CANFD3_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD3_MANUAL_DATA_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD3_MANUAL_DATA_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD3_MANUAL_DATA_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD3_MANUAL_DATA_SYNC_JUMP,
#endif
};

static canfd_extended_cfg_t g_canfd3_extend_cfg =
{
  .p_afl                 = g_canfd3_afl_table,
  .txmb_txi_enable       =
    RZV2H_CANFD_TXMB_MASK(CONFIG_RZV2H_CANFD3_TXMB_NUM),
  .error_interrupts      = RZV2H_CANFD_ERROR_INTERRUPTS(3),
  .p_data_timing         = &g_canfd3_timing_data,
#ifdef CONFIG_RZV2H_CANFD3_DELAY_COMPENSATION
  .delay_compensation    = 1,
#else
  .delay_compensation    = 0,
#endif
  .p_global_cfg          = &g_canfd_global_cfg,
  .txmb_merge_mode_enable = CANFD_TXMB_MERGE_MODE_DISABLE,
  .p_reg                 = (void *)R_CANFD,
};

static can_cfg_t g_canfd3_cfg =
{
  .channel      = 3,
  .p_bit_timing = &g_canfd3_timing,
  .p_callback   = rzv2h_canfd_fsp_callback,
  .p_context    = &g_canfd3_priv,
  .p_extend     = &g_canfd3_extend_cfg,
  .ipl          = CONFIG_RZV2H_CANFD3_INT_PRIORITY,
  .error_irq    = (IRQn_Type)CONFIG_RZV2H_CANFD3_ERR_INTSEL,
  .rx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD3_REC_INTSEL,
  .tx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD3_TRX_INTSEL,
};

static can_instance_t g_canfd3_hal_inst =
{
  .p_ctrl = (can_ctrl_t *)&g_canfd3_ctrl,
  .p_cfg  = &g_canfd3_cfg,
};

static struct rzv2h_canfd_priv_s g_canfd3_priv =
{
  .baud         = CONFIG_RZV2H_CANFD3_AUTO_NORM_RATE,
  .baud_data    = CONFIG_RZV2H_CANFD3_AUTO_DATA_RATE,
  .sample_point      = CONFIG_RZV2H_CANFD3_AUTO_SAMPLE_POINT,
#ifdef CONFIG_RZV2H_CANFD3_USE_MANUAL_BAUDRATE
  .use_manual_timing = true,
#else
  .use_manual_timing = false,
#endif
  .started           = false,
  .pin_config   = &g_rzv2h_canfd3_pin_config,
  .p_reg        = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE,
  .hal_inst     = &g_canfd3_hal_inst,
  .err_irq_event = RZV2H_IRQSEL_CANFD_ERR3,
  .rx_irq_event  = RZV2H_IRQSEL_CANFD_REC3,
  .tx_irq_event  = RZV2H_IRQSEL_CANFD_TRX3,
  .p_fsp_global = &g_canfd_global_cfg,
  .afl_usage    = &g_canfd3_afl_usage,
  .rx_fifo_type = CONFIG_RZV2H_CANFD3_RX_FIFO_TYPE,
  .txmb_num     = CONFIG_RZV2H_CANFD3_TXMB_NUM,
#ifndef CONFIG_RZV2H_CANFD_SOCKET
  .dev          =
  {
    .cd_ops  = &g_rzv2h_canfd_ops,
    .cd_priv = &g_canfd3_priv,
  },
#endif
};
#endif

#ifdef CONFIG_RZV2H_CANFD4

static struct rzv2h_canfd_pin_config_s g_rzv2h_canfd4_pin_config =
{
  .tx_pin = BOARD_CANFD4_TX_GPIO,
  .rx_pin = BOARD_CANFD4_RX_GPIO,
#ifdef CONFIG_RZV2H_CANFD4_STB_PIN
  .stb     = true,
  .stb_pin = BOARD_CANFD4_STB_GPIO,
#endif
};

static canfd_instance_ctrl_t g_canfd4_ctrl;

static can_bit_timing_cfg_t g_canfd4_timing =
{
#ifdef CONFIG_RZV2H_CANFD4_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD4_MANUAL_NORM_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD4_MANUAL_NORM_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD4_MANUAL_NORM_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD4_MANUAL_NORM_SYNC_JUMP,
#endif
};

static can_bit_timing_cfg_t g_canfd4_timing_data =
{
#ifdef CONFIG_RZV2H_CANFD4_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD4_MANUAL_DATA_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD4_MANUAL_DATA_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD4_MANUAL_DATA_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD4_MANUAL_DATA_SYNC_JUMP,
#endif
};

static canfd_extended_cfg_t g_canfd4_extend_cfg =
{
  .p_afl                 = g_canfd4_afl_table,
  .txmb_txi_enable       =
    RZV2H_CANFD_TXMB_MASK(CONFIG_RZV2H_CANFD4_TXMB_NUM),
  .error_interrupts      = RZV2H_CANFD_ERROR_INTERRUPTS(4),
  .p_data_timing         = &g_canfd4_timing_data,
#ifdef CONFIG_RZV2H_CANFD4_DELAY_COMPENSATION
  .delay_compensation    = 1,
#else
  .delay_compensation    = 0,
#endif
  .p_global_cfg          = &g_canfd_global_cfg,
  .txmb_merge_mode_enable = CANFD_TXMB_MERGE_MODE_DISABLE,
  .p_reg                 = (void *)R_CANFD,
};

static can_cfg_t g_canfd4_cfg =
{
  .channel      = 4,
  .p_bit_timing = &g_canfd4_timing,
  .p_callback   = rzv2h_canfd_fsp_callback,
  .p_context    = &g_canfd4_priv,
  .p_extend     = &g_canfd4_extend_cfg,
  .ipl          = CONFIG_RZV2H_CANFD4_INT_PRIORITY,
  .error_irq    = (IRQn_Type)CONFIG_RZV2H_CANFD4_ERR_INTSEL,
  .rx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD4_REC_INTSEL,
  .tx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD4_TRX_INTSEL,
};

static can_instance_t g_canfd4_hal_inst =
{
  .p_ctrl = (can_ctrl_t *)&g_canfd4_ctrl,
  .p_cfg  = &g_canfd4_cfg,
};

static struct rzv2h_canfd_priv_s g_canfd4_priv =
{
  .baud         = CONFIG_RZV2H_CANFD4_AUTO_NORM_RATE,
  .baud_data    = CONFIG_RZV2H_CANFD4_AUTO_DATA_RATE,
  .sample_point      = CONFIG_RZV2H_CANFD4_AUTO_SAMPLE_POINT,
#ifdef CONFIG_RZV2H_CANFD4_USE_MANUAL_BAUDRATE
  .use_manual_timing = true,
#else
  .use_manual_timing = false,
#endif
  .started           = false,
  .pin_config   = &g_rzv2h_canfd4_pin_config,
  .p_reg        = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE,
  .hal_inst     = &g_canfd4_hal_inst,
  .err_irq_event = RZV2H_IRQSEL_CANFD_ERR4,
  .rx_irq_event  = RZV2H_IRQSEL_CANFD_REC4,
  .tx_irq_event  = RZV2H_IRQSEL_CANFD_TRX4,
  .p_fsp_global = &g_canfd_global_cfg,
  .afl_usage    = &g_canfd4_afl_usage,
  .rx_fifo_type = CONFIG_RZV2H_CANFD4_RX_FIFO_TYPE,
  .txmb_num     = CONFIG_RZV2H_CANFD4_TXMB_NUM,
#ifndef CONFIG_RZV2H_CANFD_SOCKET
  .dev          =
  {
    .cd_ops  = &g_rzv2h_canfd_ops,
    .cd_priv = &g_canfd4_priv,
  },
#endif
};
#endif

#ifdef CONFIG_RZV2H_CANFD5

static struct rzv2h_canfd_pin_config_s g_rzv2h_canfd5_pin_config =
{
  .tx_pin = BOARD_CANFD5_TX_GPIO,
  .rx_pin = BOARD_CANFD5_RX_GPIO,
#ifdef CONFIG_RZV2H_CANFD5_STB_PIN
  .stb     = true,
  .stb_pin = BOARD_CANFD5_STB_GPIO,
#endif
};

static canfd_instance_ctrl_t g_canfd5_ctrl;

static can_bit_timing_cfg_t g_canfd5_timing =
{
#ifdef CONFIG_RZV2H_CANFD5_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD5_MANUAL_NORM_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD5_MANUAL_NORM_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD5_MANUAL_NORM_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD5_MANUAL_NORM_SYNC_JUMP,
#endif
};

static can_bit_timing_cfg_t g_canfd5_timing_data =
{
#ifdef CONFIG_RZV2H_CANFD5_USE_MANUAL_BAUDRATE
  .baud_rate_prescaler        = CONFIG_RZV2H_CANFD5_MANUAL_DATA_PRESCALER,
  .time_segment_1             = CONFIG_RZV2H_CANFD5_MANUAL_DATA_SEGMENT_1,
  .time_segment_2             = CONFIG_RZV2H_CANFD5_MANUAL_DATA_SEGMENT_2,
  .synchronization_jump_width = CONFIG_RZV2H_CANFD5_MANUAL_DATA_SYNC_JUMP,
#endif
};

static canfd_extended_cfg_t g_canfd5_extend_cfg =
{
  .p_afl                 = g_canfd5_afl_table,
  .txmb_txi_enable       =
    RZV2H_CANFD_TXMB_MASK(CONFIG_RZV2H_CANFD5_TXMB_NUM),
  .error_interrupts      = RZV2H_CANFD_ERROR_INTERRUPTS(5),
  .p_data_timing         = &g_canfd5_timing_data,
#ifdef CONFIG_RZV2H_CANFD5_DELAY_COMPENSATION
  .delay_compensation    = 1,
#else
  .delay_compensation    = 0,
#endif
  .p_global_cfg          = &g_canfd_global_cfg,
  .txmb_merge_mode_enable = CANFD_TXMB_MERGE_MODE_DISABLE,
  .p_reg                 = (void *)R_CANFD,
};

static can_cfg_t g_canfd5_cfg =
{
  .channel      = 5,
  .p_bit_timing = &g_canfd5_timing,
  .p_callback   = rzv2h_canfd_fsp_callback,
  .p_context    = &g_canfd5_priv,
  .p_extend     = &g_canfd5_extend_cfg,
  .ipl          = CONFIG_RZV2H_CANFD5_INT_PRIORITY,
  .error_irq    = (IRQn_Type)CONFIG_RZV2H_CANFD5_ERR_INTSEL,
  .rx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD5_REC_INTSEL,
  .tx_irq       = (IRQn_Type)CONFIG_RZV2H_CANFD5_TRX_INTSEL,
};

static can_instance_t g_canfd5_hal_inst =
{
  .p_ctrl = (can_ctrl_t *)&g_canfd5_ctrl,
  .p_cfg  = &g_canfd5_cfg,
};

static struct rzv2h_canfd_priv_s g_canfd5_priv =
{
  .baud         = CONFIG_RZV2H_CANFD5_AUTO_NORM_RATE,
  .baud_data    = CONFIG_RZV2H_CANFD5_AUTO_DATA_RATE,
  .sample_point      = CONFIG_RZV2H_CANFD5_AUTO_SAMPLE_POINT,
#ifdef CONFIG_RZV2H_CANFD5_USE_MANUAL_BAUDRATE
  .use_manual_timing = true,
#else
  .use_manual_timing = false,
#endif
  .started           = false,
  .pin_config   = &g_rzv2h_canfd5_pin_config,
  .p_reg        = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE,
  .hal_inst     = &g_canfd5_hal_inst,
  .err_irq_event = RZV2H_IRQSEL_CANFD_ERR5,
  .rx_irq_event  = RZV2H_IRQSEL_CANFD_REC5,
  .tx_irq_event  = RZV2H_IRQSEL_CANFD_TRX5,
  .p_fsp_global = &g_canfd_global_cfg,
  .afl_usage    = &g_canfd5_afl_usage,
  .rx_fifo_type = CONFIG_RZV2H_CANFD5_RX_FIFO_TYPE,
  .txmb_num     = CONFIG_RZV2H_CANFD5_TXMB_NUM,
#ifndef CONFIG_RZV2H_CANFD_SOCKET
  .dev          =
  {
    .cd_ops  = &g_rzv2h_canfd_ops,
    .cd_priv = &g_canfd5_priv,
  },
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_canfd_baud_to_timing
 *
 * Description:
 *   Convert a baud rate to FSP bit timing configuration.
 *
 *   Algorithm: sweep prescaler 1..RZV2H_NBRP_MAX, for each compute
 *   TQ/TSEG1/TSEG2 from the desired sample point, range-check, and keep
 *   the best fit (minimum bitrate error). Early exit on exact match.
 *
 * Input Parameters:
 *   baud               - Bit rate in bit/s
 *   sample_point       - Sample point in percent
 *   is_data            - true for the data phase, false for the nominal
 *                        phase
 *   delay_compensation - true if transceiver delay compensation is enabled
 *   timing             - Bit timing found, written on success
 *
 * Returned Value:
 *   OK on success, -EINVAL if an argument is out of range or no timing fits
 *
 ****************************************************************************/

static int rzv2h_canfd_baud_to_timing(uint32_t baud,
                                      uint32_t sample_point,
                                      bool is_data,
                                      bool delay_compensation,
                                      can_bit_timing_cfg_t *timing)
{
  uint32_t prescaler;
  uint32_t bit_clock;
  uint32_t tq;
  uint32_t tseg1;
  uint32_t tseg2;
  uint32_t sjw;
  uint32_t actual_bitrate;
  uint32_t delta;

  /* Best result tracking */

  uint32_t best_prescaler = 0;
  uint32_t best_tseg1     = 0;
  uint32_t best_tseg2     = 0;
  uint32_t best_sjw       = 1;
  uint32_t best_delta     = UINT32_MAX;

  if (baud == 0 || timing == NULL || sample_point < 60 || sample_point > 99)
    {
      return -EINVAL;
    }

  /* Sweep all prescaler values to find best fit */

  for (prescaler = 1; prescaler <= RZV2H_NBRP_MAX; prescaler++)
    {
      bit_clock = RZV2H_CANFD_CLK_HZ / prescaler;

      /* Minimum 8 time quanta per bit required by CAN spec */

      if (bit_clock < (8 * baud))
        {
          break;
        }

      /* Calculate time quanta from bit clock and requested bitrate */

      tq = (bit_clock + baud / 2) / baud;

      /* Calculate TSEG1 from sample point */

      tseg1 = (tq * sample_point + 50) / 100;
      if (tseg1 > 0)
        {
          tseg1--;
        }

      /* TSEG2 = remaining time quanta (minus Sync_Seg = 1 Tq) */

      tseg2 = tq - (tseg1 + 1);

      /* Enforce TSEG2 >= 2 (CAN requirement) */

      if (tseg2 < 2)
        {
          uint32_t borrow = 2 - tseg2;

          if (tseg1 >= borrow)
            {
              tseg1 -= borrow;
              tseg2 += borrow;
            }
          else
            {
              continue; /* Cannot satisfy TSEG2 >= 2 */
            }
        }

      /* Range check */

      if (is_data)
        {
          if (tseg1 > RZV2H_DTSEG1_MAX || tseg2 > RZV2H_DTSEG2_MAX ||
              prescaler > RZV2H_DBRP_MAX)
            {
              continue;
            }
        }
      else
        {
          if (tseg1 > RZV2H_NTSEG1_MAX || tseg2 > RZV2H_NTSEG2_MAX)
            {
              continue;
            }
        }

      /* Delay compensation constraint: when enabled, data rate
       * parameters are only valid for prescaler <= 2.
       * Ensures data time quantum is small enough for delay measurement
       */

      if (is_data && delay_compensation && prescaler > 2)
        {
          continue;
        }

      /* Synchronization jump width */

      sjw = (tseg2 >= 10) ? 4 : 1;

      /* Calculate actual bitrate and error */

      actual_bitrate = bit_clock / tq;
      delta = (actual_bitrate > baud) ?
              (actual_bitrate - baud) : (baud - actual_bitrate);

      /* Track best fit (minimum bitrate error) */

      if (delta < best_delta)
        {
          best_prescaler = prescaler;
          best_tseg1     = tseg1;
          best_tseg2     = tseg2;
          best_sjw       = sjw;
          best_delta     = delta;

          /* Early exit on exact match */

          if (delta == 0)
            {
              break;
            }
        }
    }

  if (best_delta == UINT32_MAX)
    {
      return -EINVAL;
    }

  timing->baud_rate_prescaler        = best_prescaler;
  timing->time_segment_1             = best_tseg1;
  timing->time_segment_2             = best_tseg2;
  timing->synchronization_jump_width = best_sjw;

  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_timing_check
 *
 * Description:
 *   Check one bit timing against the "Bit Timing Conditions" in the HW
 *   manual: the field ranges, TSEG1 > TSEG2 >= SJW (nominal) or
 *   TSEG1 >= TSEG2 >= SJW (data), and 8-385 Tq (nominal) or 5-49 Tq
 *   (data) in one bit.
 *
 * Input Parameters:
 *   t       - Bit timing to check
 *   is_data - true for the data phase, false for the nominal phase
 *
 * Returned Value:
 *   OK if the timing is allowed, -EINVAL otherwise
 *
 ****************************************************************************/

static int rzv2h_canfd_timing_check(const can_bit_timing_cfg_t *t,
                                    bool is_data)
{
  const char *phase  = is_data ? "data" : "nominal";
  uint32_t brp_max   = is_data ? RZV2H_DBRP_MAX : RZV2H_NBRP_MAX;
  uint32_t tseg1_min = is_data ? RZV2H_DTSEG1_MIN : RZV2H_NTSEG1_MIN;
  uint32_t tseg1_max = is_data ? RZV2H_DTSEG1_MAX : RZV2H_NTSEG1_MAX;
  uint32_t tseg2_min = is_data ? RZV2H_DTSEG2_MIN : RZV2H_NTSEG2_MIN;
  uint32_t tseg2_max = is_data ? RZV2H_DTSEG2_MAX : RZV2H_NTSEG2_MAX;
  uint32_t sjw_max   = is_data ? RZV2H_DSJW_MAX : RZV2H_NSJW_MAX;
  uint32_t tq_min    = is_data ? RZV2H_DTQ_MIN : RZV2H_NTQ_MIN;
  uint32_t tq_max    = is_data ? RZV2H_DTQ_MAX : RZV2H_NTQ_MAX;
  uint32_t quanta;

  if (t->baud_rate_prescaler < 1 || t->baud_rate_prescaler > brp_max ||
      t->time_segment_1 < tseg1_min || t->time_segment_1 > tseg1_max ||
      t->time_segment_2 < tseg2_min || t->time_segment_2 > tseg2_max ||
      t->synchronization_jump_width < 1 ||
      t->synchronization_jump_width > sjw_max)
    {
      canerr("ERROR: %s bit timing out of range\n", phase);
      return -EINVAL;
    }

  /* Nominal: TSEG1 > TSEG2. Data: TSEG1 >= TSEG2. Both: TSEG2 >= SJW. */

  if ((is_data ? t->time_segment_1 < t->time_segment_2 :
                 t->time_segment_1 <= t->time_segment_2) ||
      t->time_segment_2 < t->synchronization_jump_width)
    {
      canerr("ERROR: %s bit timing needs TSEG1 %s TSEG2 >= SJW\n",
             phase, is_data ? ">=" : ">");
      return -EINVAL;
    }

  quanta = 1u + t->time_segment_1 + t->time_segment_2;
  if (quanta < tq_min || quanta > tq_max)
    {
      canerr("ERROR: %s bit has %lu Tq, allowed %lu-%lu\n", phase,
             (unsigned long)quanta, (unsigned long)tq_min,
             (unsigned long)tq_max);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_bittiming_clocks
 *
 * Description:
 *   Return the clock cycles in one bit of 't': BRP clocks per Tq times
 *   (1 + TSEG1 + TSEG2) Tq, where 1 is the sync segment.
 *
 * Input Parameters:
 *   t - Bit timing
 *
 * Returned Value:
 *   Clock cycles in one bit
 *
 ****************************************************************************/

static uint32_t rzv2h_canfd_bittiming_clocks(const can_bit_timing_cfg_t *t)
{
  return t->baud_rate_prescaler *
         (1u + t->time_segment_1 + t->time_segment_2);
}

/****************************************************************************
 * Name: rzv2h_canfd_afl_ch_rules
 *
 * Description:
 *   Return the number of AFL rules allocated to a given channel.
 *
 * Input Parameters:
 *   channel - CANFD channel number (0-5)
 *
 * Returned Value:
 *   Number of AFL rules of the channel, 0 for any other channel number
 *
 ****************************************************************************/

static uint32_t rzv2h_canfd_afl_ch_rules(uint32_t channel)
{
  switch (channel)
    {
      case 0:
        return CANFD_CFG_AFL_CH0_RULE_NUM;

      case 1:
        return CANFD_CFG_AFL_CH1_RULE_NUM;

      case 2:
        return CANFD_CFG_AFL_CH2_RULE_NUM;

      case 3:
        return CANFD_CFG_AFL_CH3_RULE_NUM;

      case 4:
        return CANFD_CFG_AFL_CH4_RULE_NUM;

      case 5:
        return CANFD_CFG_AFL_CH5_RULE_NUM;

      default:
        return 0;
    }
}

#ifdef RZV2H_CANFD_HAVE_FILTER_IOCTL
/****************************************************************************
 * Name: rzv2h_canfd_afl_count_used
 *
 * Description:
 *   Count the AFL entries of this channel that belong to a filter.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   Number of AFL entries in use
 *
 ****************************************************************************/

static uint32_t rzv2h_canfd_afl_count_used(struct rzv2h_canfd_priv_s *priv)
{
  struct rzv2h_canfd_afl_usage_s *p_ch_afl_usage = priv->afl_usage;
  uint32_t count = 0;
  uint32_t i;

  for (i = 0; i < p_ch_afl_usage->num_rules; i++)
    {
      if (p_ch_afl_usage->owner[i] != 0)
        {
          count++;
        }
    }

  return count;
}

/****************************************************************************
 * Name: rzv2h_canfd_afl_free_entry
 *
 * Description:
 *   Find the first free AFL entry of this channel, from entry 'start' on.
 *
 * Input Parameters:
 *   priv  - Channel private data
 *   start - First entry to look at
 *
 * Returned Value:
 *   Index of the free entry, or -1 if no entry from 'start' on is free
 *
 ****************************************************************************/

static int32_t rzv2h_canfd_afl_free_entry(struct rzv2h_canfd_priv_s *priv,
                                          uint32_t start)
{
  struct rzv2h_canfd_afl_usage_s *p_ch_afl_usage = priv->afl_usage;
  uint32_t i;

  for (i = start; i < p_ch_afl_usage->num_rules; i++)
    {
      if (p_ch_afl_usage->owner[i] == 0)
        {
          return (int32_t)i;
        }
    }

  return -1;
}

/****************************************************************************
 * Name: rzv2h_canfd_afl_max_id_count_per_entry
 *
 * Description:
 *   Return the maximum number of consecutive IDs, starting at 'low_id',
 *   that can share one AFL entry (one ID and mask) without going past
 *   'high_id'.
 *
 * Input Parameters:
 *   low_id  - First ID
 *   high_id - Last ID that may be covered (high_id >= low_id)
 *
 * Returned Value:
 *   Number of IDs, a power of 2
 *
 ****************************************************************************/

static uint32_t rzv2h_canfd_afl_max_id_count_per_entry(uint32_t low_id,
                                                       uint32_t high_id)
{
  uint32_t max_id_count = 1;
  uint32_t next_max_id_count;
  uint64_t last_id;
  bool     low_id_starts_on_multiple;
  bool     fits_in_range;

  for (; ; )
    {
      next_max_id_count = max_id_count << 1;

      low_id_starts_on_multiple = (low_id % next_max_id_count) == 0;

      last_id       = (uint64_t)low_id + next_max_id_count - 1;
      fits_in_range = last_id <= high_id;

      if (!low_id_starts_on_multiple || !fits_in_range)
        {
          break;
        }

      max_id_count = next_max_id_count;
    }

  return max_id_count;
}

/****************************************************************************
 * Name: rzv2h_canfd_afl_range_entry_count
 *
 * Description:
 *   Return the number of AFL entries that CAN_FILTER_RANGE 'low_id' ..
 *   'high_id' needs: one per group of IDs from
 *   rzv2h_canfd_afl_max_id_count_per_entry().
 *
 * Input Parameters:
 *   low_id  - Lower ID of the range
 *   high_id - Upper ID of the range (high_id >= low_id)
 *
 * Returned Value:
 *   Number of AFL entries
 *
 ****************************************************************************/

static uint32_t rzv2h_canfd_afl_range_entry_count(uint32_t low_id,
                                                  uint32_t high_id)
{
  uint32_t entry_count = 0;
  uint32_t first_id = low_id;

  while (first_id <= high_id)
    {
      first_id += rzv2h_canfd_afl_max_id_count_per_entry(first_id, high_id);
      entry_count++;
    }

  return entry_count;
}

/****************************************************************************
 * Name: rzv2h_canfd_afl_base_entry
 *
 * Description:
 *   Compute the global hardware AFL entry offset for this channel.
 *   CH0 starts at 0, CH1 starts after CH0's rules, etc.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   Number of the channel's first entry in the whole AFL
 *
 ****************************************************************************/

static uint32_t rzv2h_canfd_afl_base_entry(struct rzv2h_canfd_priv_s *priv)
{
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  uint32_t base = 0;
  uint32_t i;

  for (i = 0; i < channel; i++)
    {
      base += rzv2h_canfd_afl_ch_rules(i);
    }

  return base;
}

#endif

/****************************************************************************
 * Name: rzv2h_canfd_set_filter_accept_all
 *
 * Description:
 *   Fill one AFL entry with accept-all rule (mask=0 means don't care).
 *   id/frame_type/id_mode are not set — mask=0 makes hardware ignore them.
 *   Matching frames are stored in the FIFO(s) given by fifo_flags.
 *
 * Input Parameters:
 *   entry      - AFL entry to fill
 *   fifo_flags - FIFO(s) that store the matching frames
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_set_filter_accept_all(canfd_afl_entry_t *entry,
                                              canfd_rx_fifo_t fifo_flags)
{
  *entry = (canfd_afl_entry_t)
    {
      .mask.mask_id          = RZV2H_AFL_MASK_ID_NOT_USED,
      .mask.mask_frame_type  = RZV2H_AFL_MASK_FRAME_TYPE_NOT_USED,
      .mask.mask_id_mode     = RZV2H_AFL_MASK_NOT_USED,
      .destination.minimum_dlc = CANFD_MINIMUM_DLC_0,
      .destination.rx_buffer   = CANFD_RX_MB_NONE,
      .destination.fifo_select_flags = fifo_flags,
    };
}

/****************************************************************************
 * Name: rzv2h_canfd_set_filter_disable
 *
 * Description:
 *   Fill one AFL entry that is not in use. Every entry in the channel's
 *   rule range takes part in the AFL search, so the entry must still hold
 *   a rule. The rule only matches extended ID 0x1FFFFFFF and has no
 *   destination (no RX message buffer and no FIFO), so a matching frame is
 *   not stored. GAFLLB = 1 prevents this rule from dropping a 0x1FFFFFFF
 *   frame that a filter in a higher entry accepts.
 *
 * Input Parameters:
 *   entry - AFL entry to fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_set_filter_disable(canfd_afl_entry_t *entry)
{
  *entry = (canfd_afl_entry_t)
    {
      .id_u32         = RZV2H_AFL_ID_MASK_EXT | RZV2H_AFL_ID_EXT |
                        R_CANFD_CFDGAFL_ID_GAFLLB_Msk,
      .mask.mask_id      = RZV2H_AFL_ID_MASK_EXT,
      .mask.mask_id_mode = CAN_ID_MODE_EXTENDED,
      .destination.rx_buffer   = CANFD_RX_MB_NONE,
      .destination.fifo_select_flags = RZV2H_CANFD_RX_FIFO_NONE,
    };
}

#ifdef RZV2H_CANFD_HAVE_FILTER_IOCTL
/****************************************************************************
 * Name: rzv2h_canfd_set_filter_rule
 *
 * Description:
 *   Fill one AFL entry with a filter rule: accept the frames whose ID,
 *   compared on the bits set in 'mask', equals 'id', in standard or
 *   extended format. Data and remote frames both match, as with the
 *   accept-all rule.
 *
 * Input Parameters:
 *   entry      - AFL entry to fill
 *   ext        - true for extended IDs
 *   id         - ID to match
 *   mask       - ID bits to compare
 *   fifo_flags - FIFO(s) that store the matching frames
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_set_filter_rule(canfd_afl_entry_t *entry, bool ext,
                                        uint32_t id, uint32_t mask,
                                        canfd_rx_fifo_t fifo_flags)
{
  *entry = (canfd_afl_entry_t)
    {
      .id_u32                  = id | (ext ? RZV2H_AFL_ID_EXT : 0),
      .mask_u32                = mask | RZV2H_AFL_MASK_IDMODE,
      .destination.minimum_dlc = CANFD_MINIMUM_DLC_0,
      .destination.rx_buffer   = CANFD_RX_MB_NONE,
      .destination.fifo_select_flags = fifo_flags,
    };
}
#endif

/****************************************************************************
 * Name: rzv2h_canfd_rx_fifo_flags
 *
 * Description:
 *   Return the FIFO bit (canfd_rx_fifo_t) for message priority 'prio',
 *   based on the channel's RX FIFO type.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   prio - Message priority (CAN_MSGPRIO_LOW or CAN_MSGPRIO_HIGH)
 *
 * Returned Value:
 *   The FIFO bit, or RZV2H_CANFD_RX_FIFO_NONE for an unknown RX FIFO type
 *
 ****************************************************************************/

static canfd_rx_fifo_t rzv2h_canfd_rx_fifo_flags(
                         struct rzv2h_canfd_priv_s *priv, uint8_t prio)
{
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  uint32_t slot;

  switch (priv->rx_fifo_type)
    {
      case RZV2H_CANFD_RX_FIFO_TYPE_RXFIFO:

        /* RX FIFO n, for any priority */

        return (canfd_rx_fifo_t)(1U << RZV2H_CANFD_RXFIFO_IDX(channel));

      case RZV2H_CANFD_RX_FIFO_TYPE_CFIFO:

        /* Common FIFO 3n, for any priority */

        slot = RZV2H_CANFD_CFIFO_SINGLE;
        break;

      case RZV2H_CANFD_RX_FIFO_TYPE_CFIFO_PRIO:

        /* Common FIFO 3n+1 for CAN_MSGPRIO_LOW, 3n otherwise */

        slot = (prio == CAN_MSGPRIO_LOW) ? RZV2H_CANFD_CFIFO_LOW :
                                           RZV2H_CANFD_CFIFO_HIGH;
        break;

      default:
        return RZV2H_CANFD_RX_FIFO_NONE;
    }

  return (canfd_rx_fifo_t)(1UL << (RZV2H_AFL_FDP_CFIFO_SHIFT +
                                   RZV2H_CANFD_CFIFO_IDX(channel, slot)));
}

/****************************************************************************
 * Name: rzv2h_canfd_ch_priv
 *
 * Description:
 *   Return the private data of a configured channel, or NULL.
 *
 * Input Parameters:
 *   channel - CANFD channel number (0-5)
 *
 * Returned Value:
 *   Private data of the channel, NULL if the channel is not enabled
 *
 ****************************************************************************/

static struct rzv2h_canfd_priv_s *rzv2h_canfd_ch_priv(uint32_t channel)
{
  switch (channel)
    {
#ifdef CONFIG_RZV2H_CANFD0
      case 0:
        return &g_canfd0_priv;
#endif
#ifdef CONFIG_RZV2H_CANFD1
      case 1:
        return &g_canfd1_priv;
#endif
#ifdef CONFIG_RZV2H_CANFD2
      case 2:
        return &g_canfd2_priv;
#endif
#ifdef CONFIG_RZV2H_CANFD3
      case 3:
        return &g_canfd3_priv;
#endif
#ifdef CONFIG_RZV2H_CANFD4
      case 4:
        return &g_canfd4_priv;
#endif
#ifdef CONFIG_RZV2H_CANFD5
      case 5:
        return &g_canfd5_priv;
#endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Name: rzv2h_canfd_rx_fifo_check_cfg
 *
 * Description:
 *   Check that the FIFO(s) picked by the channel's Kconfig "RX FIFO"
 *   choice are set up to receive:
 *
 *   - CONFIG_RZV2H_CANFDn_RX_FIFO_TYPE_RXFIFO:
 *     RX FIFO n is enabled (RFE), its depth is not 0 (RFDC) and it has
 *     the interrupt on (RFIE).
 *   - CONFIG_RZV2H_CANFDn_RX_FIFO_TYPE_CFIFO:
 *     Common FIFO 3n is enabled (CFE), its depth is not 0 (CFDC), it is
 *     in RX mode (CFM = 00b) and has the RX interrupt on (CFRXIE).
 *   - CONFIG_RZV2H_CANFDn_RX_PRIORITY_FIFOS:
 *     Common FIFOs 3n (high) and 3n+1 (low) both meet the Common FIFO
 *     conditions above.
 *
 *   Otherwise accepted frames never raise the RX interrupt (GRECC / REC).
 *   A FIFO with depth 0 cannot be enabled at all.
 *   Refer to "RX FIFO Configuration/Control Register n (CFDRFCCn)" and
 *   "Common FIFO Configuration/Control Register n (CFDCFCCn)" in the HW
 *   manual.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   OK if the FIFO(s) are set up to receive, -EINVAL otherwise
 *
 ****************************************************************************/

static int rzv2h_canfd_rx_fifo_check_cfg(struct rzv2h_canfd_priv_s *priv)
{
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  uint32_t rx_slots[2];
  uint32_t num_rx_slots = 0;
  uint32_t rxfifo;
  uint32_t cfg;
  uint32_t i;

  switch (priv->rx_fifo_type)
    {
      case RZV2H_CANFD_RX_FIFO_TYPE_RXFIFO:

        /* RX FIFO n */

        rxfifo = RZV2H_CANFD_RXFIFO_IDX(channel);
        cfg    = priv->p_fsp_global->rx_fifo_config[rxfifo];

        if ((cfg & R_CANFD_CFDRFCC_RFE_Msk) == 0 ||
            (cfg & R_CANFD_CFDRFCC_RFDC_Msk) == 0 ||
            (cfg & R_CANFD_CFDRFCC_RFIE_Msk) == 0)
          {
            canerr("ERROR: CANFD%lu: RX FIFO %lu must be Enabled, "
                   "Depth != 0, Interrupt Mode != Disable\n",
                   channel, rxfifo);
            return -EINVAL;
          }

        /* RX FIFO n is set up, exit now */

        return OK;

      case RZV2H_CANFD_RX_FIFO_TYPE_CFIFO:

        /* Common FIFO 3n */

        rx_slots[num_rx_slots++] = RZV2H_CANFD_CFIFO_SINGLE;
        break;

      case RZV2H_CANFD_RX_FIFO_TYPE_CFIFO_PRIO:

        /* Common FIFO 3n (high) and 3n+1 (low) */

        rx_slots[num_rx_slots++] = RZV2H_CANFD_CFIFO_HIGH;
        rx_slots[num_rx_slots++] = RZV2H_CANFD_CFIFO_LOW;
        break;

      default:
        canerr("ERROR: CANFD%lu: invalid RX FIFO type %u\n",
               channel, priv->rx_fifo_type);
        return -EINVAL;
    }

  /* Common FIFO modes: check every slot picked above */

  for (i = 0; i < num_rx_slots; i++)
    {
      uint32_t idx;

      idx = RZV2H_CANFD_CFIFO_IDX(channel, rx_slots[i]);
      cfg = priv->p_fsp_global->common_fifo_config[idx];

      if ((cfg & R_CANFD_CFDCFCC_CFE_Msk) == 0 ||
          (cfg & R_CANFD_CFDCFCC_CFDC_Msk) == 0 ||
          (cfg & R_CANFD_CFDCFCC_CFRXIE_Msk) == 0 ||
          (cfg & R_CANFD_CFDCFCC_CFM_Msk) != 0)
        {
          canerr("ERROR: CANFD%lu: Common FIFO %lu must be Enabled, "
                 "Depth != 0, Mode = Receive, RX Interrupt Enable = "
                 "Enabled\n", channel, idx);
          return -EINVAL;
        }
    }

  return OK;
}

#ifdef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_rx_deliver
 *
 * Description:
 *   Convert one received FSP frame to a SocketCAN frame and put it in the
 *   RX queue of 'priv'. Shared by the RX FIFO path (FSP callback) and the
 *   Common FIFO path (rzv2h_canfd_cfifo_rx_interrupt). Without
 *   CONFIG_NET_CAN_CANFD, a CAN FD frame is dropped: it does not fit in a
 *   struct can_frame.
 *
 * Input Parameters:
 *   priv  - Channel private data
 *   frame - Received FSP frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_rx_deliver(struct rzv2h_canfd_priv_s *priv,
                                   const can_frame_t *frame)
{
  struct canfd_frame cf;
  bool fd = (frame->options & CANFD_FRAME_OPTION_FD) != 0;
  uint8_t len = frame->data_length_code;

#ifndef RZV2H_CANFD_HAVE_FD
  if (fd)
    {
      return;
    }
#endif

  /* DLC 9-15 means 12-64 bytes in a CAN FD frame but 8 bytes in a
   * classic frame. r_canfd_mb_read() always uses the CAN FD meaning, so
   * cut a classic frame to 8 bytes.
   */

  if (!fd && len > RZV2H_CANFD_CLASSIC_MAXLEN)
    {
      len = RZV2H_CANFD_CLASSIC_MAXLEN;
    }

  memset(&cf, 0, sizeof(cf));

  cf.can_id = frame->id;
#ifdef RZV2H_CANFD_HAVE_EXTID
  if (frame->id_mode == CAN_ID_MODE_EXTENDED)
    {
      cf.can_id |= CAN_EFF_FLAG;
    }
#endif

  if (frame->type == CAN_FRAME_TYPE_REMOTE)
    {
      cf.can_id |= CAN_RTR_FLAG;
    }

  /* The FSP data_length_code is the byte count, the same as len (can_dlc
   * in a struct can_frame)
   */

  cf.len = len;

#ifdef RZV2H_CANFD_HAVE_FD
  if (frame->options & CANFD_FRAME_OPTION_BRS)
    {
      cf.flags |= CANFD_BRS;
    }

  if (frame->options & CANFD_FRAME_OPTION_ERROR)
    {
      cf.flags |= CANFD_ESI;
    }
#endif

  memcpy(cf.data, frame->data, len);

  rzv2h_canfd_sock_rxpush(priv, &cf, fd);
}
#else
/****************************************************************************
 * Name: rzv2h_canfd_rx_deliver
 *
 * Description:
 *   Convert one received FSP frame to a NuttX CAN message and pass it to
 *   the upper half. Shared by the RX FIFO path (FSP callback) and the
 *   Common FIFO path (rzv2h_canfd_cfifo_rx_interrupt). Without
 *   CONFIG_CAN_FD, a CAN FD frame is dropped.
 *
 * Input Parameters:
 *   priv  - Channel private data
 *   frame - Received FSP frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_rx_deliver(struct rzv2h_canfd_priv_s *priv,
                                   const can_frame_t *frame)
{
  struct can_hdr_s hdr;
  uint8_t data[CAN_MAXDATALEN];
  bool fd = (frame->options & CANFD_FRAME_OPTION_FD) != 0;
  uint8_t len = frame->data_length_code;

#ifndef CONFIG_CAN_FD
  /* A CAN FD frame does not fit in a classic CAN message */

  if (fd)
    {
      return;
    }
#endif

  /* DLC 9-15 means 12-64 bytes in a CAN FD frame but 8 bytes in a
   * classic frame. r_canfd_mb_read() always uses the CAN FD meaning, so
   * cut a classic frame to 8 bytes.
   */

  if (!fd && len > RZV2H_CANFD_CLASSIC_MAXLEN)
    {
      len = RZV2H_CANFD_CLASSIC_MAXLEN;
    }

  memset(&hdr, 0, sizeof(hdr));
  memset(data, 0, sizeof(data));

  hdr.ch_id = frame->id;
#ifdef CONFIG_CAN_EXTID
  hdr.ch_extid = (frame->id_mode == CAN_ID_MODE_EXTENDED) ? 1 : 0;
#endif
  hdr.ch_rtr = (frame->type == CAN_FRAME_TYPE_REMOTE) ? 1 : 0;

  /* DLC conversion.
   * NuttX ch_dlc stores the raw DLC value (0-15).
   * The FSP data_length_code is the byte count (0-64 for CANFD).
   * Convert byte count back to raw DLC.
   */

  hdr.ch_dlc = can_bytes2dlc(len);

  /* CAN FD flags.
   * ch_edl/ch_brs require CONFIG_CAN_FD in nuttx/can/can.h
   */

#ifdef CONFIG_CAN_FD
  hdr.ch_edl = fd ? 1 : 0;
  hdr.ch_brs = (frame->options & CANFD_FRAME_OPTION_BRS) ? 1 : 0;
#endif

  memcpy(data, frame->data, len);

  /* Deliver to upper-half */

  can_receive(&priv->dev, &hdr, data);
}
#endif

#ifdef RZV2H_CANFD_HAVE_ERRORS
/****************************************************************************
 * Name: rzv2h_canfd_report_rx_overflow
 *
 * Description:
 *   Send an RX overflow error frame (CAN_ERROR1_RXOVERFLOW) to the upper
 *   half of 'priv', after one of its FIFOs lost a frame.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_report_rx_overflow(struct rzv2h_canfd_priv_s *priv)
{
  rzv2h_canfd_errhdr_t hdr;
  uint8_t data[RZV2H_CANFD_ERR_DLC];

  memset(&hdr, 0, sizeof(hdr));
  memset(data, 0, sizeof(data));

  hdr.ch_error = 1;
  hdr.ch_dlc   = RZV2H_CANFD_ERR_DLC;
  hdr.ch_id    = RZV2H_CANFD_ERR_CONTROLLER;
  data[1]      = RZV2H_CANFD_ERR1_RXOVERFLOW;

  RZV2H_CANFD_REPORT_ERROR(priv, &hdr, data);
}
#endif

/****************************************************************************
 * Name: rzv2h_canfd_cfifo_clear_lost
 *
 * Description:
 *   If Common FIFO 'cfifo' lost a message (CFMLT), clear the flag and
 *   report an RX overflow to the channel that owns the FIFO.
 *
 * Input Parameters:
 *   cfifo - Common FIFO number (0-17)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_cfifo_clear_lost(uint32_t cfifo)
{
  volatile R_CANFD_Type *p_reg = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE;
#ifdef RZV2H_CANFD_HAVE_ERRORS
  struct rzv2h_canfd_priv_s *priv;
#endif

  /* No message lost in this Common FIFO, nothing to do */

  if ((p_reg->CFDCFSTS[cfifo] & R_CANFD_CFDCFSTS_CFMLT_Msk) == 0)
    {
      return;
    }

  /* Clear CFDCFSTSn.CFMLT. This also clears CFDFMSTS.CFXMLT of this FIFO,
   * and CFDGERFL.MES once no FIFO has a lost message left.
   */

  p_reg->CFDCFSTS[cfifo] = RZV2H_CANFD_CFSTS_CLR(R_CANFD_CFDCFSTS_CFMLT_Msk);

#ifdef RZV2H_CANFD_HAVE_ERRORS
  priv = rzv2h_canfd_ch_priv(RZV2H_CANFD_CFIFO_CH(cfifo));
  if (priv == NULL || !priv->started)
    {
      return;
    }

  rzv2h_canfd_report_rx_overflow(priv);
#endif
}

/****************************************************************************
 * Name: rzv2h_canfd_rxfifo_clear_lost
 *
 * Description:
 *   If RX FIFO 'rxfifo' lost a message (RFMLT), clear the flag and report
 *   an RX overflow to the channel that owns the FIFO.
 *
 * Input Parameters:
 *   rxfifo - RX FIFO number (0-7)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_rxfifo_clear_lost(uint32_t rxfifo)
{
  volatile R_CANFD_Type *p_reg = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE;
#ifdef RZV2H_CANFD_HAVE_ERRORS
  struct rzv2h_canfd_priv_s *priv;
#endif

  /* No message lost in this RX FIFO, nothing to do */

  if ((p_reg->CFDRFSTS[rxfifo] & R_CANFD_CFDRFSTS_RFMLT_Msk) == 0)
    {
      return;
    }

  p_reg->CFDRFSTS[rxfifo] =
    RZV2H_CANFD_RFSTS_CLR(R_CANFD_CFDRFSTS_RFMLT_Msk);

#ifdef RZV2H_CANFD_HAVE_ERRORS
  priv = rzv2h_canfd_ch_priv(RZV2H_CANFD_RXFIFO_CH(rxfifo));
  if (priv == NULL || !priv->started)
    {
      return;
    }

  rzv2h_canfd_report_rx_overflow(priv);
#endif
}

/****************************************************************************
 * Name: rzv2h_canfd_cfifo_rx_interrupt
 *
 * Description:
 *   REC interrupt of one channel. Used instead of the FSP
 *   canfd_common_fifo_rx_isr(), which takes the lowest pending Common FIFO
 *   of all channels but delivers it to the channel whose line fired, so
 *   frames of one channel can reach another channel's device. This
 *   handler only reads this channel's own FIFOs.
 *
 *   The high priority FIFO is drained on every interrupt; the low priority
 *   FIFO only when its own CFRXIF was set on entry.
 *
 *   RX flags are cleared before the FIFOs are read: a frame that arrives
 *   during the read can set the flag again and fire a new interrupt,
 *   instead of waiting in the FIFO with no flag set.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_cfifo_rx_interrupt(struct rzv2h_canfd_priv_s *priv)
{
  can_ctrl_t *p_ctrl = priv->hal_inst->p_ctrl;
  volatile R_CANFD_Type *p_reg = priv->p_reg;
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  uint32_t low_slot;
  uint32_t high_slot = RZV2H_CANFD_CFIFO_HIGH;
  uint32_t low_cfifo;
  uint32_t high_cfifo;
  can_frame_t frame;
  fsp_err_t ret;
  bool use_high_cfifo;
  bool low_pending;

  /* low_slot / high_slot are the slots (0..2) of the low / high FIFO in
   * this channel. use_high_cfifo tells whether a high FIFO (3n) is read
   * as well.
   */

  switch (priv->rx_fifo_type)
    {
      case RZV2H_CANFD_RX_FIFO_TYPE_CFIFO:

        /* Only Common FIFO 3n */

        low_slot       = RZV2H_CANFD_CFIFO_SINGLE;
        use_high_cfifo = false;
        break;

      case RZV2H_CANFD_RX_FIFO_TYPE_CFIFO_PRIO:

        /* Common FIFO 3n is the high FIFO, 3n+1 the low FIFO */

        low_slot       = RZV2H_CANFD_CFIFO_LOW;
        use_high_cfifo = true;
        break;

      case RZV2H_CANFD_RX_FIFO_TYPE_RXFIFO:
      default:

        /* RX FIFO n frames arrive on GRECC, never on this REC line */

        return;
    }

  /* low_cfifo / high_cfifo are the Common FIFO numbers (3n + slot) used
   * to index CFDCFSTSn.
   */

  low_cfifo  = RZV2H_CANFD_CFIFO_IDX(channel, low_slot);
  high_cfifo = RZV2H_CANFD_CFIFO_IDX(channel, high_slot);

  /* Low FIFO is read only if it raised the interrupt (CFRXIF set) */

  low_pending = (p_reg->CFDCFSTS[low_cfifo] &
                 R_CANFD_CFDCFSTS_CFRXIF_Msk) != 0;

  /* Clear the low FIFO's CFRXIF only if it will be read now so if it is
   * set while this handler runs, the interrupt fires again.
   */

  if (low_pending)
    {
      p_reg->CFDCFSTS[low_cfifo] =
        RZV2H_CANFD_CFSTS_CLR(R_CANFD_CFDCFSTS_CFRXIF_Msk);
    }

  /* The high FIFO is read on every interrupt, so clear its CFRXIF now. If
   * it is set again while this handler runs, the interrupt fires again.
   */

  if (use_high_cfifo)
    {
      p_reg->CFDCFSTS[high_cfifo] =
        RZV2H_CANFD_CFSTS_CLR(R_CANFD_CFDCFSTS_CFRXIF_Msk);
    }

  /* Empty the high FIFO, then take one low frame, and repeat */

  for (; ; )
    {
      /* Read the high FIFO until it is empty */

      if (use_high_cfifo)
        {
          while (R_CANFD_Read(p_ctrl, CANFD_RX_BUFFER_FIFO_COMMON_0 +
                              high_slot, &frame) == FSP_SUCCESS)
            {
              rzv2h_canfd_rx_deliver(priv, &frame);
            }
        }

      /* Low FIFO did not raise the interrupt: leave its frames for now */

      if (!low_pending)
        {
          break;
        }

      /* Take one frame from the low FIFO, stop once it is empty */

      ret = R_CANFD_Read(p_ctrl, CANFD_RX_BUFFER_FIFO_COMMON_0 + low_slot,
                         &frame);
      if (ret != FSP_SUCCESS)
        {
          break;
        }

      rzv2h_canfd_rx_deliver(priv, &frame);
    }

  /* If a FIFO lost a frame (CFMLT), clear it and report an RX overflow */

  rzv2h_canfd_cfifo_clear_lost(low_cfifo);
  if (use_high_cfifo)
    {
      rzv2h_canfd_cfifo_clear_lost(high_cfifo);
    }
}

/****************************************************************************
 * Name: rzv2h_canfd_interrupt
 *
 * Description:
 *   Interrupt handler for the three lines of one CANFD channel: ERR, REC
 *   (Common RX FIFO) and TRX (TX). The line that fired is found from the
 *   IRQ number. ERR and TRX go to the FSP ISRs through
 *   rzv2h_interrupt_common_handler(), which records the IRQ so the FSP
 *   ISRs can find their instance. REC goes to
 *   rzv2h_canfd_cfifo_rx_interrupt().
 *
 * Input Parameters:
 *   irq     - GIC interrupt number
 *   context - CPU context at time of interrupt (not used)
 *   arg     - Channel private data
 *
 * Returned Value:
 *   OK, or -EINVAL if irq is not one of this channel's IRQs
 *
 ****************************************************************************/

static int rzv2h_canfd_interrupt(int irq, void *context, void *arg)
{
  struct rzv2h_canfd_priv_s *priv = (struct rzv2h_canfd_priv_s *)arg;
  const can_cfg_t *cfg = priv->hal_inst->p_cfg;
  rzv2h_irqn_t fsp_irq;
  void (*fsp_isr)(void);

  UNUSED(context);

  /* Convert the GIC INTID back to the FSP IRQn_Type number */

  fsp_irq = (rzv2h_irqn_t)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  if (fsp_irq == (rzv2h_irqn_t)cfg->error_irq)
    {
      fsp_isr = canfd_error_isr;
    }
  else if (fsp_irq == (rzv2h_irqn_t)cfg->rx_irq)
    {
      /* Driver's own handler, needs no FSP IRQ context */

      rzv2h_canfd_cfifo_rx_interrupt(priv);
      return OK;
    }
  else if (fsp_irq == (rzv2h_irqn_t)cfg->tx_irq)
    {
      fsp_isr = canfd_channel_tx_isr;
    }
  else
    {
      return -EINVAL;
    }

  rzv2h_interrupt_common_handler(fsp_irq, fsp_isr);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_irq_attach
 *
 * Description:
 *   Connect this channel's ERR, REC and TRX INTSEL IRQs to their CANFD
 *   events and attach rzv2h_canfd_interrupt() to them. The IRQs stay
 *   disabled; R_CANFD_Open() enables them.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_irq_attach(struct rzv2h_canfd_priv_s *priv)
{
  const can_cfg_t *cfg = priv->hal_inst->p_cfg;
  rzv2h_irqn_t err_sel = (rzv2h_irqn_t)cfg->error_irq;
  rzv2h_irqn_t rx_sel  = (rzv2h_irqn_t)cfg->rx_irq;
  rzv2h_irqn_t tx_sel  = (rzv2h_irqn_t)cfg->tx_irq;

  int err_gic = RZV2H_FSP_TO_GIC_IRQ(err_sel);
  int rx_gic  = RZV2H_FSP_TO_GIC_IRQ(rx_sel);
  int tx_gic  = RZV2H_FSP_TO_GIC_IRQ(tx_sel);
  int ret;

  /* Keep the IRQs off until the handlers and FSP contexts are in place */

  up_disable_irq(err_gic);
  up_disable_irq(rx_gic);
  up_disable_irq(tx_gic);

  /* Connect the INTSEL IRQs to their CANFD events. All CANFD events are
   * level-sensitive; R_CANFD_Open() applies the detect type to the GIC.
   */

  ret = rzv2h_intsel_connect_event(rx_sel, priv->rx_irq_event,
                                   BSP_GIC_SPI_DETECT_LEVEL);
  if (ret < 0)
    {
      canerr("ERROR: Failed to connect RX INTSEL for CH%lu\n",
             cfg->channel);
      return ret;
    }

  ret = rzv2h_intsel_connect_event(tx_sel, priv->tx_irq_event,
                                   BSP_GIC_SPI_DETECT_LEVEL);
  if (ret < 0)
    {
      canerr("ERROR: Failed to connect TX INTSEL for CH%lu\n",
             cfg->channel);
      rzv2h_intsel_disconnect_event(rx_sel);
      return ret;
    }

  ret = rzv2h_intsel_connect_event(err_sel, priv->err_irq_event,
                                   BSP_GIC_SPI_DETECT_LEVEL);
  if (ret < 0)
    {
      canerr("ERROR: Failed to connect ERR INTSEL for CH%lu\n",
             cfg->channel);
      rzv2h_intsel_disconnect_event(tx_sel);
      rzv2h_intsel_disconnect_event(rx_sel);
      return ret;
    }

  /* Attach the handlers. The IRQs are not enabled here: R_CANFD_Open()
   * enables them and sets their FSP context and priority.
   */

  ret = irq_attach(err_gic, rzv2h_canfd_interrupt, priv);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach ERR IRQ %d: %d\n", err_gic, ret);
      goto errout_events;
    }

  ret = irq_attach(rx_gic, rzv2h_canfd_interrupt, priv);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach RX IRQ %d: %d\n", rx_gic, ret);
      goto errout_err;
    }

  ret = irq_attach(tx_gic, rzv2h_canfd_interrupt, priv);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach TX IRQ %d: %d\n", tx_gic, ret);
      goto errout_rx;
    }

  return OK;

errout_rx:
  irq_detach(rx_gic);
errout_err:
  irq_detach(err_gic);
errout_events:
  rzv2h_intsel_disconnect_event(tx_sel);
  rzv2h_intsel_disconnect_event(rx_sel);
  rzv2h_intsel_disconnect_event(err_sel);

  return ret;
}

/****************************************************************************
 * Name: rzv2h_canfd_irq_detach
 *
 * Description:
 *   Undo rzv2h_canfd_irq_attach(): disable and detach this channel's ERR,
 *   REC and TRX IRQs and disconnect their INTSEL events.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_irq_detach(struct rzv2h_canfd_priv_s *priv)
{
  const can_cfg_t *cfg = priv->hal_inst->p_cfg;

  int err_gic = RZV2H_FSP_TO_GIC_IRQ(cfg->error_irq);
  int rx_gic  = RZV2H_FSP_TO_GIC_IRQ(cfg->rx_irq);
  int tx_gic  = RZV2H_FSP_TO_GIC_IRQ(cfg->tx_irq);

  up_disable_irq(err_gic);
  up_disable_irq(rx_gic);
  up_disable_irq(tx_gic);

  irq_detach(err_gic);
  irq_detach(rx_gic);
  irq_detach(tx_gic);

  rzv2h_intsel_disconnect_event((rzv2h_irqn_t)cfg->error_irq);
  rzv2h_intsel_disconnect_event((rzv2h_irqn_t)cfg->rx_irq);
  rzv2h_intsel_disconnect_event((rzv2h_irqn_t)cfg->tx_irq);
}

/****************************************************************************
 * Name: rzv2h_canfd_global_interrupt
 *
 * Description:
 *   Interrupt handler for the two CANFD-wide IRQs: GERR (global error)
 *   goes to rzv2h_canfd_global_error(), GRECC (RX FIFO) to the FSP
 *   canfd_rx_fifo_isr().
 *
 * Input Parameters:
 *   irq     - Interrupt number
 *   context - CPU context at time of interrupt (not used)
 *   arg     - Not used
 *
 * Returned Value:
 *   OK, or -EINVAL if irq is neither the GERR nor the GRECC IRQ
 *
 ****************************************************************************/

static int rzv2h_canfd_global_interrupt(int irq, void *context, void *arg)
{
  rzv2h_irqn_t fsp_irq;
  void (*fsp_isr)(void);

  UNUSED(context);
  UNUSED(arg);

  fsp_irq = (rzv2h_irqn_t)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  if (fsp_irq == (rzv2h_irqn_t)CONFIG_RZV2H_CANFD_GERR_INTSEL)
    {
      /* Driver's own handler, needs no FSP IRQ context */

      rzv2h_canfd_global_error();
      return OK;
    }
  else if (fsp_irq == (rzv2h_irqn_t)CONFIG_RZV2H_CANFD_GREEC_INTSEL)
    {
      fsp_isr = canfd_rx_fifo_isr;
    }
  else
    {
      return -EINVAL;
    }

  rzv2h_interrupt_common_handler(fsp_irq, fsp_isr);

  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_global_irq_attach
 *
 * Description:
 *   Attach the shared GERR/GRECC selectable interrupts. Called from
 *   rzv2h_canfd_setup() before R_CANFD_Open(), which is what enables the
 *   lines at the GIC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_global_irq_attach(void)
{
  rzv2h_irqn_t gerr_sel  = (rzv2h_irqn_t)CONFIG_RZV2H_CANFD_GERR_INTSEL;
  rzv2h_irqn_t grecc_sel = (rzv2h_irqn_t)CONFIG_RZV2H_CANFD_GREEC_INTSEL;

  int gerr_gic  = RZV2H_FSP_TO_GIC_IRQ(gerr_sel);
  int grecc_gic = RZV2H_FSP_TO_GIC_IRQ(grecc_sel);
  int ret;

  if (g_canfd_global_irq_refs > 0)
    {
      g_canfd_global_irq_refs++;
      return OK;
    }

  up_disable_irq(gerr_gic);
  up_disable_irq(grecc_gic);

  ret = rzv2h_intsel_connect_event(gerr_sel, RZV2H_IRQSEL_CANFD_GERR,
                                   BSP_GIC_SPI_DETECT_LEVEL);
  if (ret < 0)
    {
      canerr("ERROR: Failed to connect GERR INTSEL\n");
      return ret;
    }

  ret = rzv2h_intsel_connect_event(grecc_sel, RZV2H_IRQSEL_CANFD_GRECC,
                                   BSP_GIC_SPI_DETECT_LEVEL);
  if (ret < 0)
    {
      canerr("ERROR: Failed to connect GRECC INTSEL\n");
      rzv2h_intsel_disconnect_event(gerr_sel);
      return ret;
    }

  ret = irq_attach(gerr_gic, rzv2h_canfd_global_interrupt, NULL);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach GERR IRQ %d: %d\n", gerr_gic, ret);
      goto errout_events;
    }

  ret = irq_attach(grecc_gic, rzv2h_canfd_global_interrupt, NULL);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach GRECC IRQ %d: %d\n", grecc_gic, ret);
      irq_detach(gerr_gic);
      goto errout_events;
    }

  g_canfd_global_irq_refs = 1;
  return OK;

errout_events:
  rzv2h_intsel_disconnect_event(grecc_sel);
  rzv2h_intsel_disconnect_event(gerr_sel);
  return ret;
}

/****************************************************************************
 * Name: rzv2h_canfd_global_irq_detach
 *
 * Description:
 *   Release the shared GERR/GRECC interrupts once the last channel has shut
 *   down: disable the lines, detach the NuttX handler and disconnect the
 *   INTSEL events.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_global_irq_detach(void)
{
  rzv2h_irqn_t gerr_sel  = (rzv2h_irqn_t)CONFIG_RZV2H_CANFD_GERR_INTSEL;
  rzv2h_irqn_t grecc_sel = (rzv2h_irqn_t)CONFIG_RZV2H_CANFD_GREEC_INTSEL;

  int gerr_gic  = RZV2H_FSP_TO_GIC_IRQ(gerr_sel);
  int grecc_gic = RZV2H_FSP_TO_GIC_IRQ(grecc_sel);

  if (g_canfd_global_irq_refs <= 0)
    {
      return;
    }

  if (--g_canfd_global_irq_refs > 0)
    {
      return;
    }

  up_disable_irq(gerr_gic);
  up_disable_irq(grecc_gic);

  irq_detach(gerr_gic);
  irq_detach(grecc_gic);

  rzv2h_intsel_disconnect_event(gerr_sel);
  rzv2h_intsel_disconnect_event(grecc_sel);
}

/****************************************************************************
 * Name: rzv2h_canfd_global_error
 *
 * Description:
 *   GERR interrupt. Used instead of the FSP canfd_error_isr(), which
 *   reports every global error on CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH and
 *   reads a NULL gp_ctrl[] when that channel is not open.
 *
 *   - A FIFO that lost a message: clear its lost flag and report an RX
 *     overflow to the channel that owns the FIFO.
 *   - Any other flag: report CAN_ERROR_CONTROLLER without detail to
 *     CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH. Nothing is reported while that
 *     channel is not open.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_global_error(void)
{
  volatile R_CANFD_Type *p_reg = (volatile R_CANFD_Type *)RZV2H_CANFD_BASE;
  uint32_t gerfl;
  uint32_t lost;
  uint32_t fifo;
#ifdef RZV2H_CANFD_HAVE_ERRORS
  struct rzv2h_canfd_priv_s *priv;
  rzv2h_canfd_errhdr_t hdr;
  uint8_t data[RZV2H_CANFD_ERR_DLC];
#endif

  /* Clear the flags read here. CFDGERFL.MES clears itself once no FIFO
   * has a lost message left.
   */

  gerfl = p_reg->CFDGERFL & RZV2H_CANFD_GERFL_FLAGS;
  if (gerfl != 0)
    {
      p_reg->CFDGERFL = RZV2H_CANFD_GERFL_CLR(gerfl);
    }

  lost = p_reg->CFDFMSTS;

  for (fifo = 0; fifo < RZV2H_CANFD_RXFIFO_NUM; fifo++)
    {
      if ((lost & (1U << (R_CANFD_CFDFMSTS_RFXMLT_Pos + fifo))) != 0)
        {
          rzv2h_canfd_rxfifo_clear_lost(fifo);
        }
    }

  for (fifo = 0; fifo < BSP_FEATURE_CANFD_NUM_COMMON_FIFOS; fifo++)
    {
      if ((lost & (1U << (R_CANFD_CFDFMSTS_CFXMLT_Pos + fifo))) != 0)
        {
          rzv2h_canfd_cfifo_clear_lost(fifo);
        }
    }

#ifdef RZV2H_CANFD_HAVE_ERRORS
  if (gerfl == 0)
    {
      return;
    }

  /* Only CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH gets these errors, as in
   * canfd_error_isr(). Drop them while that channel is not open.
   */

  priv = rzv2h_canfd_ch_priv(CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH);
  if (priv == NULL || !priv->started)
    {
      return;
    }

  /* These flags have no CAN_ERROR1_* code, so data[] stays all 0
   * (CAN_ERROR1_UNSPEC)
   */

  memset(&hdr, 0, sizeof(hdr));
  memset(data, 0, sizeof(data));

  hdr.ch_error = 1;
  hdr.ch_dlc   = RZV2H_CANFD_ERR_DLC;
  hdr.ch_id    = RZV2H_CANFD_ERR_CONTROLLER;

  RZV2H_CANFD_REPORT_ERROR(priv, &hdr, data);
#endif
}

#ifdef RZV2H_CANFD_HAVE_ERRORS
/****************************************************************************
 * Name: rzv2h_canfd_channel_error
 *
 * Description:
 *   Send one error report for the channel error flags 'error' (CFDCnERFL)
 *   to the upper half of 'priv', with every error class that applies.
 *   RX or TX warning and passive are picked from the error counters in
 *   CFDCnSTS. Bus error flags are not reported: the bus error interrupt
 *   is not enabled, and these flags show only the first bus error since
 *   they were last cleared.
 *
 * Input Parameters:
 *   priv  - Channel private data
 *   error - CFDCnERFL flags (canfd_error_t)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_channel_error(struct rzv2h_canfd_priv_s *priv,
                                      uint32_t error)
{
  uint32_t ch  = priv->hal_inst->p_cfg->channel;
  uint32_t sts = priv->p_reg->CFDC[ch].STS;
  uint32_t tec;
  uint32_t rec;
  rzv2h_canfd_errhdr_t hdr;
  uint8_t data[RZV2H_CANFD_ERR_DLC];

  tec = (sts & R_CANFD_CFDC_STS_TEC_Msk) >> R_CANFD_CFDC_STS_TEC_Pos;
  rec = (sts & R_CANFD_CFDC_STS_REC_Msk) >> R_CANFD_CFDC_STS_REC_Pos;

  memset(&hdr, 0, sizeof(hdr));
  memset(data, 0, sizeof(data));

  hdr.ch_error = 1;
  hdr.ch_dlc   = RZV2H_CANFD_ERR_DLC;

  if (error & CANFD_ERROR_CHANNEL_BUS_OFF_ENTRY)
    {
      hdr.ch_id |= RZV2H_CANFD_ERR_BUSOFF;
    }

  if (error & CANFD_ERROR_CHANNEL_BUS_OFF_RECOVERY)
    {
      hdr.ch_id |= RZV2H_CANFD_ERR_RESTARTED;
    }

  /* Error passive and error warning: report RX or TX by the counter at or
   * above the threshold. If no counter is there any more, report both.
   */

  if (error & CANFD_ERROR_CHANNEL_PASSIVE)
    {
      hdr.ch_id |= RZV2H_CANFD_ERR_CONTROLLER;
      data[1]   |= (rec >= CAN_ERROR_PASSIVE_THRESHOLD ?
                    RZV2H_CANFD_ERR1_RXPASSIVE : 0) |
                   (tec >= CAN_ERROR_PASSIVE_THRESHOLD ?
                    RZV2H_CANFD_ERR1_TXPASSIVE : 0);
      if (rec < CAN_ERROR_PASSIVE_THRESHOLD &&
          tec < CAN_ERROR_PASSIVE_THRESHOLD)
        {
          data[1] |= RZV2H_CANFD_ERR1_RXPASSIVE |
                     RZV2H_CANFD_ERR1_TXPASSIVE;
        }
    }

  if (error & CANFD_ERROR_CHANNEL_WARNING)
    {
      hdr.ch_id |= RZV2H_CANFD_ERR_CONTROLLER;
      data[1]   |= (rec >= CAN_ERROR_WARNING_THRESHOLD ?
                    RZV2H_CANFD_ERR1_RXWARNING : 0) |
                   (tec >= CAN_ERROR_WARNING_THRESHOLD ?
                    RZV2H_CANFD_ERR1_TXWARNING : 0);
      if (rec < CAN_ERROR_WARNING_THRESHOLD &&
          tec < CAN_ERROR_WARNING_THRESHOLD)
        {
          data[1] |= RZV2H_CANFD_ERR1_RXWARNING |
                     RZV2H_CANFD_ERR1_TXWARNING;
        }
    }

  if (error & CANFD_ERROR_CHANNEL_OVERLOAD)
    {
      hdr.ch_id |= RZV2H_CANFD_ERR_PROTOCOL;
      data[2]    = RZV2H_CANFD_ERR2_OVERLOAD;
    }

  /* None of the flags above: nothing to report */

  if (hdr.ch_id == 0)
    {
      return;
    }

  RZV2H_CANFD_REPORT_ERROR(priv, &hdr, data);
}
#endif

/****************************************************************************
 * Name: rzv2h_canfd_fsp_callback
 *
 * Description:
 *   Callback of the FSP CANFD ISRs canfd_rx_fifo_isr(),
 *   canfd_channel_tx_isr() and canfd_error_isr(). Report each event to the
 *   upper half of 'priv', the channel in p_args->p_context.
 *
 * Input Parameters:
 *   p_args - FSP callback arguments
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_fsp_callback(can_callback_args_t *p_args)
{
  struct rzv2h_canfd_priv_s *priv;

  if (p_args == NULL || p_args->p_context == NULL)
    {
      return;
    }

  priv = (struct rzv2h_canfd_priv_s *)p_args->p_context;

  switch (p_args->event)
    {
      /* Frame from an RX FIFO, read by canfd_rx_fifo_isr(). Common FIFO
       * frames go to rzv2h_canfd_cfifo_rx_interrupt() instead.
       */

      case CAN_EVENT_RX_COMPLETE:
        {
          rzv2h_canfd_rx_deliver(priv, &p_args->frame);
          break;
        }

      /* canfd_channel_tx_isr() clears CFDTMSTSn.TMTRF before this callback,
       * so p_args->buffer is already idle when can_txdone() asks
       * rzv2h_canfd_txready() for a free buffer.
       * Ignore a buffer >= txmb_num: rzv2h_canfd_send() only writes frames
       * to TXMB0 .. txmb_num - 1. A tx_discard bit means CANIOC_OFLUSH
       * already dropped that frame in the upper half.
       * A TX complete or TX abort report also clears the TX deadline of
       * the buffer, so tx_wdog does not fire for a frame that is already
       * done.
       */

      case CAN_EVENT_TX_COMPLETE:
        {
          if (p_args->buffer >= priv->txmb_num)
            {
              canwarn("WARNING: CANFD%lu: TX report for TXMB%lu ignored\n",
                      p_args->channel, p_args->buffer);
              break;
            }

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
          rzv2h_canfd_sock_deadline_clear(priv, p_args->buffer);
#endif

          if (priv->tx_discard & (1U << p_args->buffer))
            {
              priv->tx_discard &= ~(1U << p_args->buffer);
              break;
            }

#ifdef CONFIG_RZV2H_CANFD_SOCKET
          netdev_lower_txdone(&priv->dev);
#else
          can_txdone(&priv->dev);
#endif
          break;
        }

      case CAN_EVENT_TX_ABORTED:
        {
          if (p_args->buffer >= priv->txmb_num)
            {
              canwarn("WARNING: CANFD%lu: TX report for TXMB%lu ignored\n",
                      p_args->channel, p_args->buffer);
              break;
            }

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
          rzv2h_canfd_sock_deadline_clear(priv, p_args->buffer);
#endif

          /* If tx_discard is set, rzv2h_canfd_tx_flush() aborted the frame
           * and no txdone is needed. Otherwise bus-off recovery aborted it
           * and txdone is needed.
           */

          if (priv->tx_discard & (1U << p_args->buffer))
            {
              priv->tx_discard &= ~(1U << p_args->buffer);
              break;
            }

#ifdef CONFIG_RZV2H_CANFD_SOCKET
          netdev_lower_txdone(&priv->dev);
#else
          can_txdone(&priv->dev);
#endif
          break;
        }

      case CAN_EVENT_ERR_CHANNEL:
        {
#ifdef RZV2H_CANFD_HAVE_ERRORS
          rzv2h_canfd_channel_error(priv, p_args->error);
#endif
          break;
        }

      default:
        break;
    }
}

/* Channel start, stop and interrupts */

#ifndef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_reset
 *
 * Description:
 *   Reset the CAN device. Called before setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - CAN device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_reset(struct can_dev_s *dev)
{
  struct rzv2h_canfd_priv_s *priv = dev->cd_priv;

  /* Stop a running channel the same way as can_close() does */

  if (priv->started)
    {
      rzv2h_canfd_chan_rxint(priv, false);
      rzv2h_canfd_chan_txint(priv, false);
      rzv2h_canfd_chan_shutdown(priv);
    }
}
#endif

#ifdef RZV2H_CANFD_HAVE_RESTART
/****************************************************************************
 * Name: rzv2h_canfd_chan_reset
 *
 * Description:
 *   Put a running channel in CH_RESET with its test mode cleared, so its
 *   bit timing and AFL entries can be written. CH_RESET is entered at
 *   once; CH_HALT waits for the end of the frame on the bus, with no
 *   timeout. CH_RESET also cuts off a frame being received and clears the
 *   transmit requests, so callers first check that no TX message buffer
 *   is busy. Refer to "Channel Modes" in the HW manual.
 *
 * Input Parameters:
 *   priv      - Channel private data
 *   test_mode - Test mode of the channel, written on return
 *
 * Returned Value:
 *   OK on success, -EIO if a mode change fails
 *
 ****************************************************************************/

static int rzv2h_canfd_chan_reset(struct rzv2h_canfd_priv_s *priv,
                                  can_test_mode_t *test_mode)
{
  can_ctrl_t *p_ctrl = priv->hal_inst->p_ctrl;
  fsp_err_t err;

  *test_mode = ((canfd_instance_ctrl_t *)p_ctrl)->test_mode;

  /* CH_OPERATION -> CH_RESET. The test mode does not change, so
   * R_CANFD_ModeTransition() goes straight to CH_RESET, without CH_HALT.
   */

  err = R_CANFD_ModeTransition(p_ctrl, CAN_OPERATION_MODE_RESET,
                               *test_mode);
  if (err != FSP_SUCCESS)
    {
      return -EIO;
    }

  /* CH_RESET cleared CFDCnCTR.CTMS, but the FSP still holds the test mode
   * and writes CTMS only when the test mode changes. Set it to disabled
   * through CH_RESET -> CH_HALT -> CH_RESET, off the bus, so
   * rzv2h_canfd_chan_run() writes CTMS again.
   */

  if (*test_mode != CAN_TEST_MODE_DISABLED)
    {
      err = R_CANFD_ModeTransition(p_ctrl, CAN_OPERATION_MODE_RESET,
                                   CAN_TEST_MODE_DISABLED);
      if (err != FSP_SUCCESS)
        {
          /* Failed: back to CH_OPERATION with the old test mode */

          R_CANFD_ModeTransition(p_ctrl, CAN_OPERATION_MODE_NORMAL,
                                 *test_mode);
          return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_chan_run
 *
 * Description:
 *   Return a channel from rzv2h_canfd_chan_reset() to CH_OPERATION with
 *   'test_mode'.
 *
 * Input Parameters:
 *   priv      - Channel private data
 *   test_mode - Test mode to set
 *
 * Returned Value:
 *   OK on success, -EIO if a mode change fails
 *
 ****************************************************************************/

static int rzv2h_canfd_chan_run(struct rzv2h_canfd_priv_s *priv,
                                can_test_mode_t test_mode)
{
  fsp_err_t err;

  /* CH_RESET -> CH_OPERATION. With a test mode, R_CANFD_ModeTransition()
   * goes through CH_HALT first, the only mode where CFDCnCTR.CTMS can be
   * written.
   */

  err = R_CANFD_ModeTransition(priv->hal_inst->p_ctrl,
                               CAN_OPERATION_MODE_NORMAL, test_mode);
  return (err == FSP_SUCCESS) ? OK : -EIO;
}
#endif

/****************************************************************************
 * Name: rzv2h_canfd_chan_setup
 *
 * Description:
 *   Start one CANFD channel: configure its pins, compute the bit timing
 *   (unless manual timing is set in Kconfig), set an accept-all AFL rule,
 *   attach the IRQs and call R_CANFD_Open(). With CONFIG_CAN_LOOPBACK
 *   (character driver) or CONFIG_RZV2H_CANFD_SOCK_LOOPBACK (SocketCAN),
 *   also enter loopback mode.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_chan_setup(struct rzv2h_canfd_priv_s *priv)
{
  can_cfg_t const          *cfg  = priv->hal_inst->p_cfg;
  can_bit_timing_cfg_t     *timing = cfg->p_bit_timing;
  canfd_extended_cfg_t     *ext  =
    (canfd_extended_cfg_t *)cfg->p_extend;
  can_bit_timing_cfg_t     *timing_data = ext->p_data_timing;
  canfd_afl_entry_t *p_ch_afl = (canfd_afl_entry_t *)ext->p_afl;
  struct rzv2h_canfd_afl_usage_s *p_ch_afl_usage = priv->afl_usage;
  canfd_rx_fifo_t fifo_flags;
  uint32_t ch_num_rules;
  uint32_t channel;
  uint32_t i;
  fsp_err_t err;
  int ret;
#ifdef RZV2H_CANFD_HAVE_LOOPBACK
  can_test_mode_t test_mode;
#endif
#if defined(RZV2H_CANFD_HAVE_GLOBAL_ERR) && defined(RZV2H_CANFD_HAVE_ERRORS)
  struct rzv2h_canfd_priv_s *gerr_priv;
#endif

  caninfo("CANFD%lu: Setting up\n", priv->hal_inst->p_cfg->channel);

  /* Configure the CANFD TX and RX pins */

  rzv2h_configgpio(priv->pin_config->tx_pin);
  rzv2h_configgpio(priv->pin_config->rx_pin);

#ifdef RZV2H_CANFD_HAVE_STB
  /* Configure the transceiver STB pin, if this channel has one, and drive
   * it low (Normal mode)
   */

  if (priv->pin_config->stb)
    {
      rzv2h_configgpio(priv->pin_config->stb_pin);
      rzv2h_gpiowrite(priv->pin_config->stb_pin, false);
    }
#endif

  /* Compute the nominal and data bit timing from the bit rates in
   * Kconfig, unless CONFIG_RZV2H_CANFDn_USE_MANUAL_BAUDRATE is set.
   */

  if (!priv->use_manual_timing)
    {
      bool delay_comp = ext->delay_compensation;

      ret = rzv2h_canfd_baud_to_timing(priv->baud,
                                       priv->sample_point,
                                       false, delay_comp, timing);
      if (ret < 0)
        {
          canerr("ERROR: Invalid nominal baud rate %lu\n",
                 (unsigned long)priv->baud);
          return ret;
        }

      ret = rzv2h_canfd_baud_to_timing(priv->baud_data,
                                       RZV2H_CANFD_DATA_SP(priv),
                                       true, delay_comp, timing_data);
      if (ret < 0)
        {
          canerr("ERROR: Invalid data baud rate %lu\n",
                 (unsigned long)priv->baud_data);
          return ret;
        }
    }

  /* Refuse bit timing the HW manual forbids: nothing else checks the
   * manual timing from Kconfig, and the FSP parameter checks are off.
   */

  ret = rzv2h_canfd_timing_check(timing, false);
  if (ret == OK)
    {
      ret = rzv2h_canfd_timing_check(timing_data, true);
    }

  if (ret == OK && rzv2h_canfd_bittiming_clocks(timing_data) >
                   rzv2h_canfd_bittiming_clocks(timing))
    {
      canerr("ERROR: CANFD%lu: data bit is longer than the nominal bit\n",
             priv->hal_inst->p_cfg->channel);
      ret = -EINVAL;
    }

  if (ret < 0)
    {
      return ret;
    }

  /* The RX FIFO or Common FIFO(s) selected in Kconfig must be set up to
   * receive
   */

  ret = rzv2h_canfd_rx_fifo_check_cfg(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Accept every frame until the first filter is added. This only fills
   * the AFL table in ext->p_afl; R_CANFD_Open() writes it to the AFL
   * registers.
   */

  channel      = priv->hal_inst->p_cfg->channel;
  ch_num_rules = rzv2h_canfd_afl_ch_rules(channel);
  fifo_flags   = rzv2h_canfd_rx_fifo_flags(priv, CAN_MSGPRIO_LOW);

  rzv2h_canfd_set_filter_accept_all(&p_ch_afl[0], fifo_flags);
  for (i = 1; i < ch_num_rules; i++)
    {
      rzv2h_canfd_set_filter_disable(&p_ch_afl[i]);
    }

  /* Entry 0 now holds the accept-all rule, but owner[0] stays 0. It is
   * not a filter added with CANIOC_ADD_STDFILTER or CANIOC_ADD_EXTFILTER.
   */

  memset(p_ch_afl_usage->owner, 0,
         p_ch_afl_usage->num_rules * sizeof(p_ch_afl_usage->owner[0]));

  /* Attach the IRQ handlers first; R_CANFD_Open() enables the IRQs */

  ret = rzv2h_canfd_global_irq_attach();
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach global CANFD IRQs\n");
      return ret;
    }

  ret = rzv2h_canfd_irq_attach(priv);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach IRQs for CH%lu\n",
             priv->hal_inst->p_cfg->channel);
      rzv2h_canfd_global_irq_detach();
      return ret;
    }

  /* Start with no TX report to drop. Cleared before R_CANFD_Open()
   * enables the TX interrupt, so no callback can see the old value.
   */

  priv->tx_discard = 0;

  err = R_CANFD_Open(priv->hal_inst->p_ctrl, cfg);
  if (err != FSP_SUCCESS)
    {
      canerr("ERROR: R_CANFD_Open failed for CH%lu: %d\n",
             priv->hal_inst->p_cfg->channel, err);
      rzv2h_canfd_irq_detach(priv);
      rzv2h_canfd_global_irq_detach();
      return -EIO;
    }

  priv->started = true;

#ifdef RZV2H_CANFD_HAVE_LOOPBACK
  /* Enter loopback mode.
   * Internal: TX is internally routed to RX, no external wiring.
   * External: TX output is routed to RX at the pin level.
   */

  ret = rzv2h_canfd_chan_reset(priv, &test_mode);
  if (ret == OK)
    {
#ifdef CONFIG_RZV2H_CANFD_LOOPBACK_INTERNAL
      ret = rzv2h_canfd_chan_run(priv, CAN_TEST_MODE_LOOPBACK_INTERNAL);
#else
      ret = rzv2h_canfd_chan_run(priv, CAN_TEST_MODE_LOOPBACK_EXTERNAL);
#endif
    }

  if (ret < 0)
    {
      canerr("ERROR: Failed to enter loopback mode for CH%lu: %d\n",
             priv->hal_inst->p_cfg->channel, ret);
      R_CANFD_Close(priv->hal_inst->p_ctrl);
      rzv2h_canfd_irq_detach(priv);
      rzv2h_canfd_global_irq_detach();
      priv->started = false;
      return ret;
    }

#endif

#if defined(RZV2H_CANFD_HAVE_GLOBAL_ERR) && defined(RZV2H_CANFD_HAVE_ERRORS)
  /* Global errors other than RX overflow are reported only on
   * CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH, so warn while it is not open
   */

  gerr_priv = rzv2h_canfd_ch_priv(CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH);
  if (gerr_priv != priv && !gerr_priv->started)
    {
      canwarn("WARNING: CANFD%lu: global errors other than RX overflow are "
              "reported on CANFD%d, which is not open. Open CANFD%d first "
              "or change CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH\n",
              priv->hal_inst->p_cfg->channel,
              CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH,
              CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_chan_shutdown
 *
 * Description:
 *   Stop one CANFD channel: stop reception, discard the frames left in its
 *   FIFOs, call R_CANFD_Close(), put the transceiver in standby and detach
 *   its IRQs.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_chan_shutdown(struct rzv2h_canfd_priv_s *priv)
{
  can_ctrl_t *p_ctrl = priv->hal_inst->p_ctrl;
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  irqstate_t flags;
  fsp_err_t err;

  caninfo("CANFD%lu: Shutting down\n", channel);

#if defined(RZV2H_CANFD_HAVE_GLOBAL_ERR) && defined(RZV2H_CANFD_HAVE_ERRORS)
  /* Closing CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH stops the reports of global
   * errors other than RX overflow for the channels that stay open
   */

  if (channel == CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH &&
      g_canfd_global_irq_refs > 1)
    {
      canwarn("WARNING: CANFD%d closed while other channels are open: "
              "their global errors other than RX overflow are no longer "
              "reported\n", CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH);
    }
#endif

  /* No RX interrupt may read the FIFOs while they are emptied here */

  flags = enter_critical_section();

  /* Stop reception first: CH_RESET is entered at once. Then empty this
   * channel's FIFOs. A frame left in RX FIFO n would make
   * canfd_rx_fifo_isr() read a NULL gp_ctrl[] after R_CANFD_Close(), and
   * one left in a Common FIFO would come out after the next open.
   */

  R_CANFD_ModeTransition(p_ctrl, CAN_OPERATION_MODE_RESET,
                         ((canfd_instance_ctrl_t *)p_ctrl)->test_mode);
  rzv2h_canfd_rx_flush(priv);

  err = R_CANFD_Close(p_ctrl);
  if (err != FSP_SUCCESS)
    {
      canerr("ERROR: CANFD%lu: R_CANFD_Close failed: %d\n", channel, err);
    }

  /* R_CANFD_Close() went through channel reset or global reset, which
   * clears CFDTMCn.TMTR and CFDTMSTSn.TMTRF without a TX report. A
   * tx_discard bit still set (can_close() stopped waiting for TX) would
   * drop the first real report of that buffer after the next open. Refer
   * to "TX Message Buffer Control Register n (CFDTMCn)" and "TX Message
   * Buffer Status Register n (CFDTMSTSn)" in the HW manual.
   */

  priv->tx_discard = 0;
  priv->started    = false;

  leave_critical_section(flags);

#ifdef RZV2H_CANFD_HAVE_STB
  /* Put the transceiver in standby (STB high) */

  if (priv->pin_config->stb)
    {
      rzv2h_gpiowrite(priv->pin_config->stb_pin, true);
    }
#endif

  /* R_CANFD_Close() disabled this channel's IRQs. Detach the handlers and
   * disconnect the INTSEL events; the global ones only for the last
   * channel.
   */

  rzv2h_canfd_irq_detach(priv);
  rzv2h_canfd_global_irq_detach();

#ifdef RZV2H_CANFD_HAVE_GLOBAL_ERR
  /* R_CANFD_Close() of CONFIG_RZV2H_CANFD_GLOBAL_ERR_CH disables the GERR
   * IRQ. Enable it again while other channels are open, so their global
   * errors are still handled.
   */

  if (g_canfd_global_irq_refs > 0)
    {
      up_enable_irq(RZV2H_FSP_TO_GIC_IRQ(
                      (rzv2h_irqn_t)CONFIG_RZV2H_CANFD_GERR_INTSEL));
    }
#endif
}

/****************************************************************************
 * Name: rzv2h_canfd_chan_rxint
 *
 * Description:
 *   Enable or disable RX at the GIC: this channel's REC IRQ (Common FIFOs)
 *   and the GRECC IRQ (RX FIFOs). GRECC is shared, so it is disabled only
 *   when no channel has RX on.
 *
 * Input Parameters:
 *   priv   - Channel private data
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_chan_rxint(struct rzv2h_canfd_priv_s *priv,
                                   bool enable)
{
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  int rx_gic    = RZV2H_FSP_TO_GIC_IRQ(priv->hal_inst->p_cfg->rx_irq);
  int grecc_gic = RZV2H_FSP_TO_GIC_IRQ(
                    (rzv2h_irqn_t)CONFIG_RZV2H_CANFD_GREEC_INTSEL);

  if (enable)
    {
      g_canfd_rxint_mask |= (1U << channel);
      up_enable_irq(rx_gic);
      up_enable_irq(grecc_gic);
    }
  else
    {
      g_canfd_rxint_mask &= ~(1U << channel);
      up_disable_irq(rx_gic);

      if (g_canfd_rxint_mask == 0)
        {
          up_disable_irq(grecc_gic);
        }
    }
}

/****************************************************************************
 * Name: rzv2h_canfd_chan_txint
 *
 * Description:
 *   Enable or disable this channel's TRX IRQ (TX complete and TX abort) at
 *   the GIC.
 *
 * Input Parameters:
 *   priv   - Channel private data
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_chan_txint(struct rzv2h_canfd_priv_s *priv,
                                   bool enable)
{
  int tx_gic = RZV2H_FSP_TO_GIC_IRQ(priv->hal_inst->p_cfg->tx_irq);

  if (enable)
    {
      up_enable_irq(tx_gic);
    }
  else
    {
      up_disable_irq(tx_gic);
    }
}

/* can_ops_s implementation */

#ifndef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_setup
 *
 * Description:
 *   can_ops_s setup(): start the channel, see rzv2h_canfd_chan_setup().
 *
 * Input Parameters:
 *   dev - CAN device
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_setup(struct can_dev_s *dev)
{
  return rzv2h_canfd_chan_setup(dev->cd_priv);
}

/****************************************************************************
 * Name: rzv2h_canfd_shutdown
 *
 * Description:
 *   can_ops_s shutdown(): stop the channel, see
 *   rzv2h_canfd_chan_shutdown().
 *
 * Input Parameters:
 *   dev - CAN device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_shutdown(struct can_dev_s *dev)
{
  rzv2h_canfd_chan_shutdown(dev->cd_priv);
}

/****************************************************************************
 * Name: rzv2h_canfd_rxint
 *
 * Description:
 *   can_ops_s rxint(): see rzv2h_canfd_chan_rxint().
 *
 * Input Parameters:
 *   dev    - CAN device
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_rxint(struct can_dev_s *dev, bool enable)
{
  rzv2h_canfd_chan_rxint(dev->cd_priv, enable);
}

/****************************************************************************
 * Name: rzv2h_canfd_txint
 *
 * Description:
 *   can_ops_s txint(): see rzv2h_canfd_chan_txint().
 *
 * Input Parameters:
 *   dev    - CAN device
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_txint(struct can_dev_s *dev, bool enable)
{
  rzv2h_canfd_chan_txint(dev->cd_priv, enable);
}

/****************************************************************************
 * Name: rzv2h_canfd_send
 *
 * Description:
 *   Send one CAN message. Converts NuttX can_msg_s to FSP can_frame_t
 *   and calls R_CANFD_Write() for the TX message buffer picked by
 *   rzv2h_canfd_txmb_next().
 *
 * Input Parameters:
 *   dev - CAN device
 *   msg - Message to send
 *
 * Returned Value:
 *   OK on success, -EBUSY if no TX message buffer is free, -EIO if the
 *   channel is not open or R_CANFD_Write() fails
 *
 ****************************************************************************/

static int rzv2h_canfd_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct rzv2h_canfd_priv_s *priv = dev->cd_priv;
  can_frame_t frame;
  fsp_err_t err;
  int mb;

  if (!priv->started)
    {
      return -EIO;
    }

  mb = rzv2h_canfd_txmb_next(priv);
  if (mb < 0)
    {
      return -EBUSY;
    }

  memset(&frame, 0, sizeof(frame));

  frame.id      = msg->cm_hdr.ch_id;
#ifdef CONFIG_CAN_EXTID
  frame.id_mode = msg->cm_hdr.ch_extid ? CAN_ID_MODE_EXTENDED
                                        : CAN_ID_MODE_STANDARD;
#else
  frame.id_mode = CAN_ID_MODE_STANDARD;
#endif
  frame.type    = msg->cm_hdr.ch_rtr ? CAN_FRAME_TYPE_REMOTE
                                      : CAN_FRAME_TYPE_DATA;

  /* frame.data_length_code is a byte count, not a DLC value */

  frame.data_length_code = can_dlc2bytes(msg->cm_hdr.ch_dlc);

  frame.options = 0;
#ifdef CONFIG_CAN_FD
  if (msg->cm_hdr.ch_edl)
    {
      frame.options |= CANFD_FRAME_OPTION_FD;
    }

  if (msg->cm_hdr.ch_brs)
    {
      frame.options |= CANFD_FRAME_OPTION_BRS;
    }
#endif

  memcpy(frame.data, msg->cm_data, frame.data_length_code);

  err = R_CANFD_Write(priv->hal_inst->p_ctrl, (uint32_t)mb, &frame);
  if (err != FSP_SUCCESS)
    {
      canerr("ERROR: R_CANFD_Write failed for CH%lu: %d\n",
             priv->hal_inst->p_cfg->channel, err);
      return -EIO;
    }

  return OK;
}

#endif

/****************************************************************************
 * Name: rzv2h_canfd_txmb_busy
 *
 * Description:
 *   Return true if TX message buffer 'mb' of this channel cannot take a new
 *   frame: a transmit request is pending, or canfd_channel_tx_isr() has not
 *   cleared the previous result yet (RZV2H_CANFD_TMSTS_BUSY).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   mb   - TX message buffer number of this channel
 *
 * Returned Value:
 *   true if the buffer is busy, false otherwise
 *
 ****************************************************************************/

static bool rzv2h_canfd_txmb_busy(struct rzv2h_canfd_priv_s *priv,
                                  uint32_t mb)
{
  uint32_t idx = RZV2H_CANFD_TXMB_IDX(priv->hal_inst->p_cfg->channel, mb);

  return (priv->p_reg->CFDTMSTS[idx] & RZV2H_CANFD_TMSTS_BUSY) != 0;
}

/****************************************************************************
 * Name: rzv2h_canfd_txmb_next
 *
 * Description:
 *   Return the TX message buffer for the next frame: the one just above the
 *   highest busy buffer, or TXMB0 when all are idle. A free buffer below a
 *   busy one is not used, so that the older frame goes out first with both
 *   CFDGCFG.TPRI settings:
 *
 *   - Buffer Number: the lowest buffer number with a transmit request goes
 *     first. With the rule above, that is the oldest frame.
 *   - Message ID: the lowest ID goes first. For the same ID, the lowest
 *     buffer number (the oldest frame) goes first.
 *
 *   Refer to "Transmission Priority" in the HW manual.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   0 .. txmb_num - 1 - the TX message buffer to load
 *   -1                - TXMB(txmb_num - 1) is busy, wait
 *
 ****************************************************************************/

static int rzv2h_canfd_txmb_next(struct rzv2h_canfd_priv_s *priv)
{
  int mb;

  for (mb = priv->txmb_num - 1; mb >= 0; mb--)
    {
      if (rzv2h_canfd_txmb_busy(priv, mb))
        {
          if (mb + 1 < priv->txmb_num)
            {
              return mb + 1;
            }
          else
            {
              return -1;
            }
        }
    }

  return 0;
}

#ifndef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message:
 *   rzv2h_canfd_txmb_next() has a TX message buffer for it.
 *
 * Input Parameters:
 *   dev - CAN device
 *
 * Returned Value:
 *   true if another TX message can be accepted, false otherwise
 *
 ****************************************************************************/

static bool rzv2h_canfd_txready(struct can_dev_s *dev)
{
  return rzv2h_canfd_txmb_next(dev->cd_priv) >= 0;
}

/****************************************************************************
 * Name: rzv2h_canfd_txempty
 *
 * Description:
 *   Return true if no TX message buffer is busy, so rzv2h_canfd_txmb_next()
 *   starts again at TXMB0.
 *
 * Input Parameters:
 *   dev - CAN device
 *
 * Returned Value:
 *   true if no TX message buffer is busy, false otherwise
 *
 ****************************************************************************/

static bool rzv2h_canfd_txempty(struct can_dev_s *dev)
{
  return rzv2h_canfd_txmb_next(dev->cd_priv) == 0;
}

#endif

#ifdef RZV2H_CANFD_HAVE_FILTER_IOCTL
/****************************************************************************
 * Name: rzv2h_canfd_afl_write_entry
 *
 * Description:
 *   Write 'entry' to AFL entry 'global_offset' through the CFDGAFLECTR
 *   page window. Call it inside a critical section: all channels share
 *   CFDGAFLECTR. The channel must be in CH_RESET or CH_HALT. Refer to
 *   "Entering Entries in the AFL" in the HW manual.
 *
 * Input Parameters:
 *   priv          - Channel private data
 *   global_offset - Entry number in the whole AFL
 *   entry         - AFL entry to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_afl_write_entry(struct rzv2h_canfd_priv_s *priv,
                                        uint32_t global_offset,
                                        const canfd_afl_entry_t *entry)
{
  volatile R_CANFD_Type *p_reg = priv->p_reg;
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  uint32_t page = global_offset / RZV2H_AFL_PAGE_SIZE;
  uint32_t idx  = global_offset % RZV2H_AFL_PAGE_SIZE;
  uint32_t p0;

  /* CFDGAFLP0n.GAFLPTR = channel, as R_CANFD_Open() sets it:
   * canfd_rx_fifo_isr() finds the channel of a frame from it.
   */

  p0  = entry->destination_u32[0] &
        ~(R_CANFD_CFDGAFL_P0_GAFLPTR_Msk | RZV2H_AFL_P0_GAFLSRD_MSK);
  p0 |= (channel << R_CANFD_CFDGAFL_P0_GAFLPTR_Pos) &
        R_CANFD_CFDGAFL_P0_GAFLPTR_Msk;

  /* Select the page and enable AFL writes */

  p_reg->CFDGAFLECTR = page | RZV2H_AFLDAE;

  p_reg->CFDGAFL[idx].ID = entry->id_u32;
  p_reg->CFDGAFL[idx].M  = entry->mask_u32;
  p_reg->CFDGAFL[idx].P0 = p0;
  p_reg->CFDGAFL[idx].P1 = entry->destination_u32[1];

  /* Disable AFL writes */

  p_reg->CFDGAFLECTR = 0;
}
#endif /* RZV2H_CANFD_HAVE_FILTER_IOCTL */

#ifndef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_bittiming_calc
 *
 * Description:
 *   Validate one CANIOC_SET_BITTIMING request and convert it to an FSP
 *   can_bit_timing_cfg_t.
 *
 *   Checks, nominal (is_data = false) / data (is_data = true):
 *   - The bit rate must divide the CANFD clock exactly:
 *     BRP = RZV2H_CANFD_CLK_HZ / (baud * (1 + TSEG1 + TSEG2)).
 *   - The result must pass rzv2h_canfd_timing_check().
 *
 * Input Parameters:
 *   bt      - CANIOC_SET_BITTIMING request
 *   is_data - true for the data phase, false for the nominal phase
 *   out     - FSP bit timing, written on success
 *
 * Returned Value:
 *   OK on success, -EINVAL if the request is not valid
 *
 ****************************************************************************/

static int rzv2h_canfd_bittiming_calc(const struct canioc_bittiming_s *bt,
                                      bool is_data,
                                      can_bit_timing_cfg_t *out)
{
  can_bit_timing_cfg_t t;
  uint64_t bit_clocks;
  int ret;

  if (bt->bt_baud == 0 || bt->bt_baud > RZV2H_CANFD_CLK_HZ)
    {
      return -EINVAL;
    }

  bit_clocks = (uint64_t)bt->bt_baud *
               (1u + bt->bt_tseg1 + bt->bt_tseg2);

  if (bit_clocks > RZV2H_CANFD_CLK_HZ ||
      (RZV2H_CANFD_CLK_HZ % bit_clocks) != 0)
    {
      return -EINVAL;
    }

  t.baud_rate_prescaler        = RZV2H_CANFD_CLK_HZ / (uint32_t)bit_clocks;
  t.time_segment_1             = bt->bt_tseg1;
  t.time_segment_2             = bt->bt_tseg2;
  t.synchronization_jump_width = bt->bt_sjw;

  ret = rzv2h_canfd_timing_check(&t, is_data);
  if (ret == OK)
    {
      *out = t;
    }

  return ret;
}

/****************************************************************************
 * Name: rzv2h_canfd_apply_bittiming
 *
 * Description:
 *   Write the stored nominal and data timing into a running channel.
 *   CFDCnNCFG and CFDCnDCFG can only be written in CH_RESET or CH_HALT, so
 *   the channel goes CH_RESET -> (write) -> CH_OPERATION with
 *   rzv2h_canfd_chan_reset() and rzv2h_canfd_chan_run(). CFDCnNCFG,
 *   CFDCnDCFG and CFDCnFDCFG.TDCO get the values R_CANFD_Open() writes.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   OK on success, -EIO if a mode change fails
 *
 ****************************************************************************/

static int rzv2h_canfd_apply_bittiming(struct rzv2h_canfd_priv_s *priv)
{
  const can_cfg_t *cfg = priv->hal_inst->p_cfg;
  const can_bit_timing_cfg_t *t = cfg->p_bit_timing;
  volatile R_CANFD_Type *p_reg = priv->p_reg;
  can_test_mode_t test_mode;
  uint32_t ch = cfg->channel;
  int ret;
#ifdef CONFIG_CAN_FD
  const can_bit_timing_cfg_t *td =
    ((canfd_extended_cfg_t *)cfg->p_extend)->p_data_timing;
  uint32_t tdco;
#endif

  ret = rzv2h_canfd_chan_reset(priv, &test_mode);
  if (ret < 0)
    {
      return ret;
    }

  p_reg->CFDC[ch].NCFG =
    (((t->baud_rate_prescaler - 1) & R_CANFD_CFDC_NCFG_NBRP_Msk) <<
     R_CANFD_CFDC_NCFG_NBRP_Pos) |
    ((t->time_segment_1 - 1u) << R_CANFD_CFDC_NCFG_NTSEG1_Pos) |
    ((t->time_segment_2 - 1u) << R_CANFD_CFDC_NCFG_NTSEG2_Pos) |
    ((t->synchronization_jump_width - 1u) << R_CANFD_CFDC_NCFG_NSJW_Pos);

#ifdef CONFIG_CAN_FD
  p_reg->CFDC2[ch].DCFG =
    (((td->baud_rate_prescaler - 1) & R_CANFD_CFDC2_DCFG_DBRP_Msk) <<
     R_CANFD_CFDC2_DCFG_DBRP_Pos) |
    ((td->time_segment_1 - 1u) << R_CANFD_CFDC2_DCFG_DTSEG1_Pos) |
    ((td->time_segment_2 - 1u) << R_CANFD_CFDC2_DCFG_DTSEG2_Pos) |
    ((td->synchronization_jump_width - 1u) << R_CANFD_CFDC2_DCFG_DSJW_Pos);

  tdco = td->time_segment_1;
  if (tdco > UINT8_MAX)
    {
      tdco = UINT8_MAX;
    }

  p_reg->CFDC2[ch].FDCFG =
    (p_reg->CFDC2[ch].FDCFG & ~R_CANFD_CFDC2_FDCFG_TDCO_Msk) |
    (tdco << R_CANFD_CFDC2_FDCFG_TDCO_Pos);
#endif

  return rzv2h_canfd_chan_run(priv, test_mode);
}

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_set_bittiming
 *
 * Description:
 *   CANIOC_SET_BITTIMING: set the nominal (or, with CONFIG_CAN_FD and
 *   bt->type == CAN_BITTIMING_DATA, the data phase) bit timing.
 *
 *   1. Validate and convert (rzv2h_canfd_bittiming_calc). A rejected
 *      request changes nothing.
 *   2. With CONFIG_CAN_FD, keep data bit time <= nominal bit time, and the
 *      data prescaler <= 2 with transceiver delay compensation.
 *   3. While open, apply both phases to the hardware
 *      (rzv2h_canfd_apply_bittiming).
 *   4. Remember the rate in priv->baud or priv->baud_data, so the next
 *      open() with automatic timing keeps it (rzv2h_canfd_setup()).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Pointer to struct canioc_bittiming_s
 *
 * Returned Value:
 *   OK on success, -EINVAL if the request is not valid, -EBUSY if a TX
 *   message buffer is in use, -EIO if the hardware update fails
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_set_bittiming(struct rzv2h_canfd_priv_s *priv,
                                           unsigned long arg)
{
  struct canioc_bittiming_s *bt =
    (struct canioc_bittiming_s *)(uintptr_t)arg;
  can_bit_timing_cfg_t *nominal =
    (can_bit_timing_cfg_t *)priv->hal_inst->p_cfg->p_bit_timing;
  can_bit_timing_cfg_t *target = nominal;
  can_bit_timing_cfg_t newt;
  can_bit_timing_cfg_t old;
  bool is_data = false;
  int ret;
#ifdef CONFIG_CAN_FD
  can_bit_timing_cfg_t *data = ((canfd_extended_cfg_t *)
    priv->hal_inst->p_cfg->p_extend)->p_data_timing;
#endif

  if (bt == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_CAN_FD
  if (bt->type == CAN_BITTIMING_DATA)
    {
      is_data = true;
      target  = data;
    }
#endif

  ret = rzv2h_canfd_bittiming_calc(bt, is_data, &newt);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_CAN_FD
  if (rzv2h_canfd_bittiming_clocks(is_data ? &newt : data) >
      rzv2h_canfd_bittiming_clocks(is_data ? nominal : &newt))
    {
      return -EINVAL;
    }

  /* With transceiver delay compensation the data prescaler must be 2 or
   * less. Refer to "Baud Rate" in the HW manual.
   */

  if (is_data && newt.baud_rate_prescaler > 2 &&
      ((canfd_extended_cfg_t *)
       priv->hal_inst->p_cfg->p_extend)->delay_compensation)
    {
      return -EINVAL;
    }
#endif

  /* rzv2h_canfd_txmb_next() is 0 only when no TX buffer is busy */

  if (priv->started && rzv2h_canfd_txmb_next(priv) != 0)
    {
      return -EBUSY;
    }

  old     = *target;
  *target = newt;

  if (priv->started)
    {
      ret = rzv2h_canfd_apply_bittiming(priv);
      if (ret < 0)
        {
          *target = old;
          return ret;
        }
    }

  if (is_data)
    {
      priv->baud_data = bt->bt_baud;
    }
  else
    {
      priv->baud = bt->bt_baud;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_get_bittiming
 *
 * Description:
 *   Get current nominal or data phase bit timing.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Pointer to struct canioc_bittiming_s, filled on return
 *
 * Returned Value:
 *   OK on success, -EINVAL if arg is NULL
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_get_bittiming(struct rzv2h_canfd_priv_s *priv,
                                           unsigned long arg)
{
  struct canioc_bittiming_s *bt =
    (struct canioc_bittiming_s *)(uintptr_t)arg;
  can_bit_timing_cfg_t *timing;
  uint32_t quanta;

  if (bt == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_CAN_FD
  if (bt->type == CAN_BITTIMING_DATA)
    {
      timing = ((canfd_extended_cfg_t *)
                priv->hal_inst->p_cfg->p_extend)->p_data_timing;
    }
  else
#endif
    {
      timing = (can_bit_timing_cfg_t *)
                priv->hal_inst->p_cfg->p_bit_timing;
    }

  quanta = timing->time_segment_1 + timing->time_segment_2 + 1;

  if (timing->baud_rate_prescaler > 0 && quanta > 0)
    {
      bt->bt_baud = RZV2H_CANFD_CLK_HZ /
                    (timing->baud_rate_prescaler * quanta);
    }
  else
    {
      bt->bt_baud = 0;
    }

  bt->bt_tseg1 = timing->time_segment_1;
  bt->bt_tseg2 = timing->time_segment_2;
  bt->bt_sjw   = timing->synchronization_jump_width;
  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_set_connmodes
 *
 * Description:
 *   CANIOC_SET_CONNMODES: select loopback and silent, mapped onto the
 *   channel test modes of the HW manual (CFDCnCTR.CTMS):
 *
 *     loopback silent  mode
 *        0       0     CAN_TEST_MODE_DISABLED
 *        1       0     LOOPBACK_EXTERNAL
 *        0       1     LISTEN (listen-only)
 *        1       1     LOOPBACK_INTERNAL
 *
 *   The channel restarts through CH_RESET (rzv2h_canfd_chan_reset()), not
 *   CH_HALT, so a bus stuck at dominant cannot hang the CPU.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Pointer to struct canioc_connmodes_s
 *
 * Returned Value:
 *   OK on success, -EINVAL if arg is NULL, -EPERM if the channel is not
 *   open, -EBUSY if a TX message buffer is in use, -EIO if the mode change
 *   fails
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_set_connmodes(struct rzv2h_canfd_priv_s *priv,
                                           unsigned long arg)
{
  struct canioc_connmodes_s *modes =
    (struct canioc_connmodes_s *)(uintptr_t)arg;
  can_test_mode_t mode;
  can_test_mode_t old;
  int ret;

  if (modes == NULL)
    {
      return -EINVAL;
    }

  if (!priv->started)
    {
      return -EPERM;
    }

  if (modes->bm_loopback)
    {
      mode = modes->bm_silent ? CAN_TEST_MODE_LOOPBACK_INTERNAL :
                                CAN_TEST_MODE_LOOPBACK_EXTERNAL;
    }
  else
    {
      mode = modes->bm_silent ? CAN_TEST_MODE_LISTEN :
                                CAN_TEST_MODE_DISABLED;
    }

  if (mode == ((canfd_instance_ctrl_t *)priv->hal_inst->p_ctrl)->test_mode)
    {
      return OK;
    }

  /* CH_RESET clears the transmit requests */

  if (rzv2h_canfd_txmb_next(priv) != 0)
    {
      return -EBUSY;
    }

  ret = rzv2h_canfd_chan_reset(priv, &old);
  if (ret < 0)
    {
      return ret;
    }

  return rzv2h_canfd_chan_run(priv, mode);
}

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_get_connmodes
 *
 * Description:
 *   CANIOC_GET_CONNMODES: report the test mode as loopback and silent, the
 *   inverse of the table in rzv2h_canfd_ioctl_set_connmodes().
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Pointer to struct canioc_connmodes_s, filled on return
 *
 * Returned Value:
 *   OK on success, -EINVAL if arg is NULL
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_get_connmodes(struct rzv2h_canfd_priv_s *priv,
                                           unsigned long arg)
{
  struct canioc_connmodes_s *modes =
    (struct canioc_connmodes_s *)(uintptr_t)arg;
  can_test_mode_t mode;

  if (modes == NULL)
    {
      return -EINVAL;
    }

  mode = ((canfd_instance_ctrl_t *)priv->hal_inst->p_ctrl)->test_mode;

  modes->bm_loopback = (mode == CAN_TEST_MODE_LOOPBACK_EXTERNAL ||
                        mode == CAN_TEST_MODE_LOOPBACK_INTERNAL) ? 1 : 0;
  modes->bm_silent   = (mode == CAN_TEST_MODE_LISTEN ||
                        mode == CAN_TEST_MODE_LOOPBACK_INTERNAL) ? 1 : 0;
  return OK;
}

#endif

/****************************************************************************
 * Name: rzv2h_canfd_rx_flush
 *
 * Description:
 *   Discard every frame waiting in this channel's hardware RX FIFO(s):
 *   read until empty.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_rx_flush(struct rzv2h_canfd_priv_s *priv)
{
  can_ctrl_t *p_ctrl = priv->hal_inst->p_ctrl;
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  can_frame_t frame;
  uint32_t rxfifo;
  uint32_t buffer;

  if (!priv->started)
    {
      return;
    }

  switch (priv->rx_fifo_type)
    {
      case RZV2H_CANFD_RX_FIFO_TYPE_RXFIFO:

        /* RX FIFO n: this channel's RX FIFO, then its FSP buffer number
         * for R_CANFD_Read()
         */

        rxfifo = RZV2H_CANFD_RXFIFO_IDX(channel);
        buffer = CANFD_RX_BUFFER_FIFO_0 + rxfifo;

        while (R_CANFD_Read(p_ctrl, buffer, &frame) == FSP_SUCCESS)
          {
          }
        break;

      case RZV2H_CANFD_RX_FIFO_TYPE_CFIFO:

        /* Common FIFO 3n: slot 0 of this channel. R_CANFD_Read() takes the
         * slot and adds the channel's 3n itself.
         */

        buffer = CANFD_RX_BUFFER_FIFO_COMMON_0 + RZV2H_CANFD_CFIFO_SINGLE;

        while (R_CANFD_Read(p_ctrl, buffer, &frame) == FSP_SUCCESS)
          {
          }
        break;

      case RZV2H_CANFD_RX_FIFO_TYPE_CFIFO_PRIO:

        /* Common FIFO 3n (high), then 3n+1 (low), given as slots as above */

        buffer = CANFD_RX_BUFFER_FIFO_COMMON_0 + RZV2H_CANFD_CFIFO_HIGH;

        while (R_CANFD_Read(p_ctrl, buffer, &frame) == FSP_SUCCESS)
          {
          }

        buffer = CANFD_RX_BUFFER_FIFO_COMMON_0 + RZV2H_CANFD_CFIFO_LOW;

        while (R_CANFD_Read(p_ctrl, buffer, &frame) == FSP_SUCCESS)
          {
          }
        break;

      default:
        break;
    }
}

#ifndef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_ioctl_iflush
 *
 * Description:
 *   Flush this channel's hardware RX FIFO(s).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Not used
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_iflush(struct rzv2h_canfd_priv_s *priv,
                                    unsigned long arg)
{
  irqstate_t flags;

  flags = enter_critical_section();
  rzv2h_canfd_rx_flush(priv);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_tx_flush
 *
 * Description:
 *   Drop the frames held in this channel's TX message buffers. A pending
 *   request is aborted with CFDTMCn.TMTAR, and the buffer's tx_discard bit
 *   makes rzv2h_canfd_fsp_callback() drop its TX report instead of calling
 *   can_txdone().
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_tx_flush(struct rzv2h_canfd_priv_s *priv)
{
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  volatile R_CANFD_Type *p_reg = priv->p_reg;
  uint32_t idx;
  uint32_t mb;
  uint8_t sts;

  if (!priv->started)
    {
      return;
    }

  for (mb = 0; mb < priv->txmb_num; mb++)
    {
      idx = RZV2H_CANFD_TXMB_IDX(channel, mb);
      sts = p_reg->CFDTMSTS[idx];
      if ((sts & RZV2H_CANFD_TMSTS_BUSY) == 0)
        {
          continue;
        }

      priv->tx_discard |= (1U << mb);

      if ((sts & R_CANFD_CFDTMSTS_TMTRM_Msk) != 0)
        {
          p_reg->CFDTMC[idx] |= R_CANFD_CFDTMC_TMTAR_Msk;
        }
    }
}

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_oflush
 *
 * Description:
 *   Flush hardware TX: drop the frames in the TX message buffers
 *   (rzv2h_canfd_tx_flush).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Not used
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_oflush(struct rzv2h_canfd_priv_s *priv,
                                    unsigned long arg)
{
  irqstate_t flags;

  flags = enter_critical_section();
  rzv2h_canfd_tx_flush(priv);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_ioflush
 *
 * Description:
 *   Flush both RX and TX hardware.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Not used
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_ioflush(struct rzv2h_canfd_priv_s *priv,
                                     unsigned long arg)
{
  irqstate_t flags;

  flags = enter_critical_section();
  rzv2h_canfd_rx_flush(priv);
  rzv2h_canfd_tx_flush(priv);
  leave_critical_section(flags);
  return OK;
}

#endif

#ifdef RZV2H_CANFD_HAVE_FILTER_IOCTL
/****************************************************************************
 * Name: rzv2h_canfd_afl_add
 *
 * Description:
 *   Add filter 'req' to this channel. It takes one AFL entry for
 *   CAN_FILTER_MASK, two for CAN_FILTER_DUAL, and for CAN_FILTER_RANGE one
 *   per group of IDs from rzv2h_canfd_afl_max_id_count_per_entry(), so
 *   exactly id1 .. id2 pass. The filter number is its first entry. The
 *   entries are written while the channel is in CH_RESET
 *   (rzv2h_canfd_chan_reset()).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   req  - Filter to add
 *
 * Returned Value:
 *   Number of the new filter on success, -EINVAL if 'req' is not valid,
 *   -ENETDOWN if the channel is not open, -ENOMEM if not enough AFL
 *   entries are free, -EBUSY if a TX message buffer is in use, -EIO if a
 *   mode change fails
 *
 ****************************************************************************/

static int rzv2h_canfd_afl_add(struct rzv2h_canfd_priv_s *priv,
                               const struct rzv2h_canfd_filter_s *req)
{
  canfd_extended_cfg_t *p_ext =
      (canfd_extended_cfg_t *)priv->hal_inst->p_cfg->p_extend;
  canfd_afl_entry_t *p_ch_afl = (canfd_afl_entry_t *)p_ext->p_afl;
  struct rzv2h_canfd_afl_usage_s *p_ch_afl_usage = priv->afl_usage;
  uint32_t id_mask = req->ext ? RZV2H_AFL_ID_MASK_EXT :
                                RZV2H_AFL_ID_MASK_STD;
  uint32_t id1 = req->id1 & id_mask;
  uint32_t id2 = req->id2 & id_mask;
  canfd_rx_fifo_t fifo_flags;
  can_test_mode_t test_mode;
  uint32_t base;
  uint32_t need;
  uint32_t size = 0;
  uint32_t id;
  uint32_t mask;
  uint32_t k;
  int32_t first;
  int32_t entry;
  irqstate_t flags;
  int ret;

  if (req->prio > CAN_MSGPRIO_HIGH)
    {
      return -EINVAL;
    }

  switch (req->type)
    {
      case CAN_FILTER_MASK:
        need = 1;
        break;

      case CAN_FILTER_DUAL:
        need = 2;
        break;

      case CAN_FILTER_RANGE:
        if (id1 > id2)
          {
            return -EINVAL;
          }

        need = rzv2h_canfd_afl_range_entry_count(id1, id2);
        break;

      default:
        return -EINVAL;                /* Not a supported filter type */
    }

  fifo_flags = rzv2h_canfd_rx_fifo_flags(priv, req->prio);
  base       = rzv2h_canfd_afl_base_entry(priv);

  flags = enter_critical_section();

  if (!priv->started)
    {
      ret = -ENETDOWN;
      goto errout;
    }

  if (p_ch_afl_usage->num_rules - rzv2h_canfd_afl_count_used(priv) < need)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* CH_RESET clears the transmit requests */

  if (rzv2h_canfd_txmb_next(priv) != 0)
    {
      ret = -EBUSY;
      goto errout;
    }

  ret = rzv2h_canfd_chan_reset(priv, &test_mode);
  if (ret < 0)
    {
      goto errout;
    }

  /* Use the lowest free entries. While no filter is set, entry 0 holds the
   * accept-all rule but is free, so the first filter replaces it.
   */

  first = rzv2h_canfd_afl_free_entry(priv, 0);
  entry = first;
  id    = id1;

  for (k = 0; k < need; k++)
    {
      switch (req->type)
        {
          case CAN_FILTER_MASK:
            mask = id2;
            break;

          case CAN_FILTER_DUAL:
            id   = (k == 0) ? id1 : id2;
            mask = id_mask;
            break;

          /* Other types were rejected above, before CH_RESET */

          case CAN_FILTER_RANGE:
          default:
            size = rzv2h_canfd_afl_max_id_count_per_entry(id, id2);
            mask = id_mask & ~(size - 1);
            break;
        }

      rzv2h_canfd_set_filter_rule(&p_ch_afl[entry], req->ext, id, mask,
                                  fifo_flags);
      p_ch_afl_usage->owner[entry] = RZV2H_AFL_OWNER(first);
      rzv2h_canfd_afl_write_entry(priv, base + entry, &p_ch_afl[entry]);

      id   += size;
      entry = rzv2h_canfd_afl_free_entry(priv, entry + 1);
    }

  ret = rzv2h_canfd_chan_run(priv, test_mode);
  if (ret == OK)
    {
      ret = first;
    }

errout:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: rzv2h_canfd_afl_del
 *
 * Description:
 *   Delete filter 'num' of this channel. Every AFL entry of the filter gets
 *   the unused rule (rzv2h_canfd_set_filter_disable()), and entry 0 gets
 *   the accept-all rule back when no filter is left. The entries are
 *   written while the channel is in CH_RESET (rzv2h_canfd_chan_reset()).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   num  - Filter number, as returned by rzv2h_canfd_afl_add()
 *   ext  - true for an extended ID filter
 *
 * Returned Value:
 *   OK on success, -EINVAL if there is no such filter, -ENETDOWN if the
 *   channel is not open, -EBUSY if a TX message buffer is in use, -EIO if
 *   a mode change fails
 *
 ****************************************************************************/

static int rzv2h_canfd_afl_del(struct rzv2h_canfd_priv_s *priv,
                               uint32_t num, bool ext)
{
  canfd_extended_cfg_t *p_ext =
      (canfd_extended_cfg_t *)priv->hal_inst->p_cfg->p_extend;
  canfd_afl_entry_t *p_ch_afl = (canfd_afl_entry_t *)p_ext->p_afl;
  struct rzv2h_canfd_afl_usage_s *p_ch_afl_usage = priv->afl_usage;
  canfd_rx_fifo_t fifo_flags;
  can_test_mode_t test_mode;
  uint32_t base;
  uint32_t i;
  irqstate_t flags;
  int ret;

  if (num >= p_ch_afl_usage->num_rules)
    {
      return -EINVAL;
    }

  base = rzv2h_canfd_afl_base_entry(priv);

  flags = enter_critical_section();

  if (!priv->started)
    {
      ret = -ENETDOWN;
      goto errout;
    }

  /* 'num' must be the first entry of a filter. Its rule holds the ID
   * format, which must be the one of the delete command.
   */

  if (p_ch_afl_usage->owner[num] != RZV2H_AFL_OWNER(num) ||
      ((p_ch_afl[num].id_u32 & RZV2H_AFL_ID_EXT) != 0) != ext)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* CH_RESET clears the transmit requests */

  if (rzv2h_canfd_txmb_next(priv) != 0)
    {
      ret = -EBUSY;
      goto errout;
    }

  ret = rzv2h_canfd_chan_reset(priv, &test_mode);
  if (ret < 0)
    {
      goto errout;
    }

  /* The entries of a filter are at its number and above */

  for (i = num; i < p_ch_afl_usage->num_rules; i++)
    {
      if (p_ch_afl_usage->owner[i] == RZV2H_AFL_OWNER(num))
        {
          rzv2h_canfd_set_filter_disable(&p_ch_afl[i]);
          p_ch_afl_usage->owner[i] = 0;
          rzv2h_canfd_afl_write_entry(priv, base + i, &p_ch_afl[i]);
        }
    }

  /* If no filters remain, restore accept-all at entry 0 */

  if (rzv2h_canfd_afl_count_used(priv) == 0)
    {
      fifo_flags = rzv2h_canfd_rx_fifo_flags(priv, CAN_MSGPRIO_LOW);
      rzv2h_canfd_set_filter_accept_all(&p_ch_afl[0], fifo_flags);
      rzv2h_canfd_afl_write_entry(priv, base, &p_ch_afl[0]);
    }

  ret = rzv2h_canfd_chan_run(priv, test_mode);

errout:
  leave_critical_section(flags);
  return ret;
}

#ifndef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_ioctl_add_stdfilter
 *
 * Description:
 *   CANIOC_ADD_STDFILTER: add a standard 11-bit ID filter
 *   (rzv2h_canfd_afl_add()).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Pointer to struct canioc_stdfilter_s
 *
 * Returned Value:
 *   Number of the new filter on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_add_stdfilter(struct rzv2h_canfd_priv_s *priv,
                                           unsigned long arg)
{
  struct canioc_stdfilter_s *sf =
    (struct canioc_stdfilter_s *)(uintptr_t)arg;
  struct rzv2h_canfd_filter_s req;

  if (sf == NULL)
    {
      return -EINVAL;
    }

  req.id1  = sf->sf_id1;
  req.id2  = sf->sf_id2;
  req.type = sf->sf_type;
  req.prio = sf->sf_prio;
  req.ext  = false;

  return rzv2h_canfd_afl_add(priv, &req);
}

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_del_stdfilter
 *
 * Description:
 *   CANIOC_DEL_STDFILTER: delete a standard ID filter
 *   (rzv2h_canfd_afl_del()).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Number of the filter to delete
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_del_stdfilter(struct rzv2h_canfd_priv_s *priv,
                                           unsigned long arg)
{
  return rzv2h_canfd_afl_del(priv, (uint32_t)arg, false);
}

#ifdef CONFIG_CAN_EXTID
/****************************************************************************
 * Name: rzv2h_canfd_ioctl_add_extfilter
 *
 * Description:
 *   CANIOC_ADD_EXTFILTER: add an extended 29-bit ID filter
 *   (rzv2h_canfd_afl_add()).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Pointer to struct canioc_extfilter_s
 *
 * Returned Value:
 *   Number of the new filter on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_add_extfilter(struct rzv2h_canfd_priv_s *priv,
                                           unsigned long arg)
{
  struct canioc_extfilter_s *xf =
    (struct canioc_extfilter_s *)(uintptr_t)arg;
  struct rzv2h_canfd_filter_s req;

  if (xf == NULL)
    {
      return -EINVAL;
    }

  req.id1  = xf->xf_id1;
  req.id2  = xf->xf_id2;
  req.type = xf->xf_type;
  req.prio = xf->xf_prio;
  req.ext  = true;

  return rzv2h_canfd_afl_add(priv, &req);
}

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_del_extfilter
 *
 * Description:
 *   CANIOC_DEL_EXTFILTER: delete an extended ID filter
 *   (rzv2h_canfd_afl_del()).
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Number of the filter to delete
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_del_extfilter(struct rzv2h_canfd_priv_s *priv,
                                           unsigned long arg)
{
  return rzv2h_canfd_afl_del(priv, (uint32_t)arg, true);
}
#endif /* CONFIG_CAN_EXTID */
#endif /* !CONFIG_RZV2H_CANFD_SOCKET */

/****************************************************************************
 * Name: rzv2h_canfd_ioctl_busoff_recovery
 *
 * Description:
 *   CANIOC_BUSOFF_RECOVERY and SIOCCANRECOVERY: force the channel out of
 *   bus-off with CFDCnCTR.RTBO. The pending transmissions are aborted
 *   first, as the HW manual asks before RTBO, so their frames are lost.
 *   Refer to "CAN Channel Bus-Off State" in the HW manual.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   arg  - Not used
 *
 * Returned Value:
 *   OK if the channel is not in bus-off (nothing to recover) or the
 *   command was issued, -EPERM if not open, -EIO if the channel is not in
 *   CH_OPERATION, -EBUSY if a transmission was not aborted in time
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl_busoff_recovery(struct rzv2h_canfd_priv_s *priv,
                                             unsigned long arg)
{
  volatile R_CANFD_Type *p_reg = priv->p_reg;
  uint32_t ch = priv->hal_inst->p_cfg->channel;
  uint32_t pending = 0;
  uint32_t idx;
  uint32_t mb;
  uint32_t sts;
  int us;

  if (!priv->started)
    {
      return -EPERM;
    }

  sts = p_reg->CFDC[ch].STS;

  if ((sts & R_CANFD_CFDC_STS_BOSTS_Msk) == 0)
    {
      return OK;
    }

  if ((sts & (R_CANFD_CFDC_STS_CRSTSTS_Msk |
              R_CANFD_CFDC_STS_CHLTSTS_Msk |
              R_CANFD_CFDC_STS_CSLPSTS_Msk)) != 0)
    {
      return -EIO;
    }

  /* Abort each pending transmission. No tx_discard bit is set, so
   * rzv2h_canfd_fsp_callback() reports the aborted frame as done.
   */

  for (mb = 0; mb < priv->txmb_num; mb++)
    {
      idx = RZV2H_CANFD_TXMB_IDX(ch, mb);
      if ((p_reg->CFDTMSTS[idx] & R_CANFD_CFDTMSTS_TMTRM_Msk) != 0)
        {
          p_reg->CFDTMC[idx] |= R_CANFD_CFDTMC_TMTAR_Msk;
          pending |= 1u << mb;
        }
    }

  /* Wait until each abort ends. The HW clears CFDTMCn.TMTR when
   * CFDTMSTSn.TMTRF gets the result, so check TMTR.
   */

  for (us = 0; pending != 0; us++)
    {
      for (mb = 0; mb < priv->txmb_num; mb++)
        {
          idx = RZV2H_CANFD_TXMB_IDX(ch, mb);
          if ((p_reg->CFDTMSTS[idx] & R_CANFD_CFDTMSTS_TMTRM_Msk) == 0)
            {
              pending &= ~(1u << mb);
            }
        }

      if (pending != 0)
        {
          if (us >= RZV2H_CANFD_ABORT_WAIT_US)
            {
              return -EBUSY;
            }

          up_udelay(1);
        }
    }

  p_reg->CFDC[ch].CTR |= R_CANFD_CFDC_CTR_RTBO_Msk;
  return OK;
}

#endif /* RZV2H_CANFD_HAVE_FILTER_IOCTL */

#ifdef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_sock_rxpush
 *
 * Description:
 *   Put one frame in the RX queue of 'priv' and tell the network stack.
 *   Called in interrupt context. When the queue is full, drop the frame
 *   and set rx_lost: rzv2h_canfd_sock_receive() then sends an RX overflow
 *   error frame.
 *
 * Input Parameters:
 *   priv  - Channel private data
 *   frame - Frame to queue (struct can_frame in the first 16 bytes when
 *           'fd' is false)
 *   fd    - true for a CAN FD frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_sock_rxpush(struct rzv2h_canfd_priv_s *priv,
                                    const struct canfd_frame *frame,
                                    bool fd)
{
  irqstate_t flags;
  uint32_t idx;

  flags = enter_critical_section();

  if (priv->rx_count >= CONFIG_RZV2H_CANFD_SOCK_RX_DEPTH)
    {
      priv->rx_lost = true;
      leave_critical_section(flags);
      return;
    }

  idx = (priv->rx_head + priv->rx_count) % CONFIG_RZV2H_CANFD_SOCK_RX_DEPTH;
  priv->rx_queue[idx].frame = *frame;
  priv->rx_queue[idx].fd    = fd;
  priv->rx_count++;

  leave_critical_section(flags);

  netdev_lower_rxready(&priv->dev);
}

#ifdef RZV2H_CANFD_HAVE_ERRORS
/****************************************************************************
 * Name: rzv2h_canfd_sock_error
 *
 * Description:
 *   Queue one error report as a SocketCAN error frame: CAN_ERR_FLAG and
 *   the error class in can_id, the detail bytes in data[].
 *
 * Input Parameters:
 *   priv - Channel private data
 *   hdr  - Error class (ch_id) and length (ch_dlc)
 *   data - CAN_ERR_DLC detail bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_sock_error(struct rzv2h_canfd_priv_s *priv,
                                   const rzv2h_canfd_errhdr_t *hdr,
                                   const uint8_t *data)
{
  struct canfd_frame cf;

  memset(&cf, 0, sizeof(cf));

  cf.can_id = CAN_ERR_FLAG | hdr->ch_id;
  cf.len    = hdr->ch_dlc;
  memcpy(cf.data, data, CAN_ERR_DLC);

  rzv2h_canfd_sock_rxpush(priv, &cf, false);
}
#endif

/****************************************************************************
 * Name: rzv2h_canfd_sock_ifup
 *
 * Description:
 *   NuttX callback: bring the interface up. Start the channel with the
 *   current bit rates and enable its RX and TX interrupts.
 *
 * Input Parameters:
 *   dev - Network device of the channel
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int rzv2h_canfd_sock_ifup(struct netdev_lowerhalf_s *dev)
{
  struct rzv2h_canfd_priv_s *priv =
    container_of(dev, struct rzv2h_canfd_priv_s, dev);
  irqstate_t flags;
  int ret;

  /* Start with an empty RX queue. R_CANFD_Open() enables the IRQs, so
   * empty it first.
   */

  flags = enter_critical_section();
  priv->rx_head  = 0;
  priv->rx_count = 0;
  priv->rx_lost  = false;
  leave_critical_section(flags);

  nxmutex_lock(&g_canfd_lock);

  ret = rzv2h_canfd_chan_setup(priv);
  if (ret == OK)
    {
      rzv2h_canfd_chan_rxint(priv, true);
      rzv2h_canfd_chan_txint(priv, true);
    }

  nxmutex_unlock(&g_canfd_lock);
  return ret;
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_ifdown
 *
 * Description:
 *   NuttX callback: bring the interface down. Disable the RX and TX
 *   interrupts and stop the channel.
 *
 * Input Parameters:
 *   dev - Network device of the channel
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int rzv2h_canfd_sock_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct rzv2h_canfd_priv_s *priv =
    container_of(dev, struct rzv2h_canfd_priv_s, dev);

  nxmutex_lock(&g_canfd_lock);

  if (priv->started)
    {
      rzv2h_canfd_chan_rxint(priv, false);
      rzv2h_canfd_chan_txint(priv, false);

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
      wd_cancel(&priv->tx_wdog);
      memset(priv->tx_deadline, 0, sizeof(priv->tx_deadline));
#endif

      rzv2h_canfd_chan_shutdown(priv);

      /* R_CANFD_Close() stopped the TX message buffers */

      rzv2h_canfd_sock_txfree(priv);
    }

  nxmutex_unlock(&g_canfd_lock);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_txfree
 *
 * Description:
 *   Free the packets held for the TX message buffers.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_sock_txfree(struct rzv2h_canfd_priv_s *priv)
{
  int mb;

  for (mb = 0; mb < priv->txmb_num; mb++)
    {
      if (priv->tx_pkt[mb] != NULL)
        {
          netpkt_free(&priv->dev, priv->tx_pkt[mb], NETPKT_TX);
          priv->tx_pkt[mb] = NULL;
        }
    }
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_reclaim
 *
 * Description:
 *   NuttX callback, called when the TX quota is used up: free the held
 *   packets once every TX message buffer is idle, so that
 *   rzv2h_canfd_txmb_next() starts again at TXMB0 with all txmb_num
 *   buffers free.
 *
 * Input Parameters:
 *   dev - Network device of the channel
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_sock_reclaim(struct netdev_lowerhalf_s *dev)
{
  struct rzv2h_canfd_priv_s *priv =
    container_of(dev, struct rzv2h_canfd_priv_s, dev);

  if (priv->started && rzv2h_canfd_txmb_next(priv) == 0)
    {
      rzv2h_canfd_sock_txfree(priv);
    }
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_fd_bytes
 *
 * Description:
 *   Round a CAN FD payload length up to the next length a DLC can code:
 *   0-8, 12, 16, 20, 24, 32, 48 or 64 bytes. R_CANFD_Write() rounds an
 *   other length down and drops the last bytes.
 *
 * Input Parameters:
 *   len - Payload length in bytes (0-64)
 *
 * Returned Value:
 *   Rounded length in bytes
 *
 ****************************************************************************/

#ifdef RZV2H_CANFD_HAVE_FD
static uint8_t rzv2h_canfd_sock_fd_bytes(uint8_t len)
{
  static const uint8_t fd_bytes[] =
  {
    8, 12, 16, 20, 24, 32, 48, 64
  };

  unsigned int i;

  for (i = 0; i < sizeof(fd_bytes); i++)
    {
      if (len <= fd_bytes[i])
        {
          return len <= 8 ? len : fd_bytes[i];
        }
    }

  return CANFD_MAX_DLEN;
}
#endif

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
/****************************************************************************
 * Name: rzv2h_canfd_sock_deadline_get
 *
 * Description:
 *   Get the TX deadline of a packet. If the socket gave a deadline
 *   (CAN_RAW_TX_DEADLINE), it is a struct timeval after the frame.
 *   Otherwise the deadline is CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE
 *   microseconds from now, or none if that is 0.
 *
 * Input Parameters:
 *   dev      - Network device of the channel
 *   pkt      - Packet holding one frame
 *   len      - Packet length in bytes
 *   deadline - Returns the deadline in CLOCK_MONOTONIC time, 0 for none
 *
 * Returned Value:
 *   Packet length without the struct timeval
 *
 ****************************************************************************/

static unsigned int rzv2h_canfd_sock_deadline_get(
                        struct netdev_lowerhalf_s *dev, netpkt_t *pkt,
                        unsigned int len, struct timespec *deadline)
{
  struct timeval tv;
  struct timespec delay;

  if (len == sizeof(struct can_frame) + sizeof(tv) ||
      len == sizeof(struct canfd_frame) + sizeof(tv))
    {
      /* Subtract the timeval size, so len is the frame length */

      len -= sizeof(tv);

      /* Copy the deadline timeval that follows the frame into tv */

      netpkt_copyout(dev, (uint8_t *)&tv, pkt, sizeof(tv), len);

      deadline->tv_sec  = tv.tv_sec;
      deadline->tv_nsec = tv.tv_usec * NSEC_PER_USEC;

      return len;
    }

  if (CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE > 0)
    {
      nxclock_gettime(CLOCK_MONOTONIC, deadline);
      clock_usec2time(&delay, CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE);
      clock_timespec_add(deadline, &delay, deadline);
    }
  else
    {
      deadline->tv_sec  = 0;
      deadline->tv_nsec = 0;
    }

  return len;
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_deadline_arm
 *
 * Description:
 *   Start tx_wdog for the earliest TX deadline, or stop it if no TX
 *   message buffer has a deadline.
 *
 * Input Parameters:
 *   priv - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_sock_deadline_arm(struct rzv2h_canfd_priv_s *priv)
{
  struct timespec *first = NULL;
  clock_t ticks;
  int mb;

  for (mb = 0; mb < priv->txmb_num; mb++)
    {
      struct timespec *ts = &priv->tx_deadline[mb];

      if ((ts->tv_sec != 0 || ts->tv_nsec != 0) &&
          (first == NULL || clock_timespec_compare(ts, first) < 0))
        {
          first = ts;
        }
    }

  if (first == NULL)
    {
      wd_cancel(&priv->tx_wdog);
      return;
    }

  clock_abstime2ticks(CLOCK_MONOTONIC, first, &ticks);
  wd_start(&priv->tx_wdog, ticks, rzv2h_canfd_sock_deadline_expiry,
           (wdparm_t)priv);
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_deadline_clear
 *
 * Description:
 *   Clear the TX deadline of a TX message buffer whose frame was sent or
 *   aborted, then call rzv2h_canfd_sock_deadline_arm() to start tx_wdog
 *   for the next deadline, or stop it if no TX message buffer has a
 *   deadline.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   mb   - TX message buffer (0 .. txmb_num - 1)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_sock_deadline_clear(struct rzv2h_canfd_priv_s *priv,
                                            uint32_t mb)
{
  irqstate_t flags;

  flags = enter_critical_section();

  priv->tx_deadline[mb].tv_sec  = 0;
  priv->tx_deadline[mb].tv_nsec = 0;
  rzv2h_canfd_sock_deadline_arm(priv);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_deadline_expiry
 *
 * Description:
 *   tx_wdog handler. Abort the frame of each TX message buffer that is
 *   still pending at its deadline, then start tx_wdog for the next
 *   deadline.
 *
 * Input Parameters:
 *   arg - Channel private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_sock_deadline_expiry(wdparm_t arg)
{
  struct rzv2h_canfd_priv_s *priv = (struct rzv2h_canfd_priv_s *)arg;
  uint32_t channel = priv->hal_inst->p_cfg->channel;
  volatile R_CANFD_Type *p_reg = priv->p_reg;
  struct timespec now;
  irqstate_t flags;
  uint32_t idx;
  int mb;

  flags = enter_critical_section();

  if (priv->started)
    {
      nxclock_gettime(CLOCK_MONOTONIC, &now);

      for (mb = 0; mb < priv->txmb_num; mb++)
        {
          struct timespec *ts = &priv->tx_deadline[mb];

          if ((ts->tv_sec == 0 && ts->tv_nsec == 0) ||
              clock_timespec_compare(ts, &now) > 0)
            {
              continue;
            }

          /* CFDTMCn.TMTAR can only be set while CFDTMCn.TMTR is set.
           * CFDTMSTSn.TMTRM shows CFDTMCn.TMTR.
           */

          idx = RZV2H_CANFD_TXMB_IDX(channel, mb);
          if ((p_reg->CFDTMSTS[idx] & R_CANFD_CFDTMSTS_TMTRM_Msk) != 0)
            {
              p_reg->CFDTMC[idx] |= R_CANFD_CFDTMC_TMTAR_Msk;
              NETDEV_TXTIMEOUTS(&priv->dev.netdev);
            }

          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }

      rzv2h_canfd_sock_deadline_arm(priv);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: rzv2h_canfd_sock_transmit
 *
 * Description:
 *   NuttX callback: send one SocketCAN frame (struct can_frame or, with
 *   CONFIG_NET_CAN_CANFD, struct canfd_frame). Load it into the TX message
 *   buffer picked by rzv2h_canfd_txmb_next() and hold the packet until
 *   rzv2h_canfd_sock_reclaim() frees it. The TX complete interrupt calls
 *   netdev_lower_txdone().
 *
 *   A packet that transmit() refuses is lost: devif_poll() releases it,
 *   and the socket has already reported the frame as sent. Holding the
 *   packets keeps the TX quota (txmb_num) at 0 from the moment the last
 *   TX message buffer is loaded until rzv2h_canfd_sock_reclaim() frees
 *   them, so transmit() is not called while rzv2h_canfd_txmb_next() has
 *   no buffer.
 *
 *   With CONFIG_NET_CAN_RAW_TX_DEADLINE, a frame whose deadline has passed
 *   is dropped, and rzv2h_canfd_sock_deadline_expiry() aborts a frame
 *   that is still pending at its deadline.
 *
 * Input Parameters:
 *   dev - Network device of the channel
 *   pkt - Packet holding one frame
 *
 * Returned Value:
 *   OK on success or if the frame is dropped, -EBUSY if no TX message
 *   buffer is free, -EINVAL for a packet that is not one frame, -EIO if
 *   the channel is not started or R_CANFD_Write() fails
 *
 ****************************************************************************/

static int rzv2h_canfd_sock_transmit(struct netdev_lowerhalf_s *dev,
                                     netpkt_t *pkt)
{
  struct rzv2h_canfd_priv_s *priv =
    container_of(dev, struct rzv2h_canfd_priv_s, dev);
  unsigned int len = netpkt_getdatalen(dev, pkt);
  uint8_t *buf = netpkt_getdata(dev, pkt);
  can_frame_t frame;
  canid_t can_id;
  fsp_err_t err;
  int mb;
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct timespec deadline;
  irqstate_t flags;
  clock_t ticks;
#endif

  if (!priv->started)
    {
      return -EIO;
    }

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  len = rzv2h_canfd_sock_deadline_get(dev, pkt, len, &deadline);
  if (deadline.tv_sec != 0 || deadline.tv_nsec != 0)
    {
      clock_abstime2ticks(CLOCK_MONOTONIC, &deadline, &ticks);

      /* 0 ticks means the deadline has passed, so drop the frame unsent */

      if (ticks == 0)
        {
          NETDEV_TXTIMEOUTS(&dev->netdev);
          netpkt_free(dev, pkt, NETPKT_TX);
          return OK;
        }
    }
#endif

  mb = rzv2h_canfd_txmb_next(priv);
  if (mb < 0)
    {
      return -EBUSY;
    }

  memset(&frame, 0, sizeof(frame));

  if (len == sizeof(struct can_frame))
    {
      struct can_frame *cf = (struct can_frame *)buf;

      if (cf->can_dlc > CAN_MAX_DLEN)
        {
          return -EINVAL;
        }

      can_id                 = cf->can_id;
      frame.data_length_code = cf->can_dlc;
      memcpy(frame.data, cf->data, cf->can_dlc);
    }
#ifdef RZV2H_CANFD_HAVE_FD
  else if (len == sizeof(struct canfd_frame))
    {
      struct canfd_frame *cf = (struct canfd_frame *)buf;

      if (cf->len > CANFD_MAX_DLEN)
        {
          return -EINVAL;
        }

      can_id        = cf->can_id;
      frame.options = CANFD_FRAME_OPTION_FD;

      if (cf->flags & CANFD_BRS)
        {
          frame.options |= CANFD_FRAME_OPTION_BRS;
        }

      if (cf->flags & CANFD_ESI)
        {
          frame.options |= CANFD_FRAME_OPTION_ERROR;
        }

      /* Copy cf->len bytes; the bytes up to the rounded length stay 0 */

      frame.data_length_code = rzv2h_canfd_sock_fd_bytes(cf->len);
      memcpy(frame.data, cf->data, cf->len);
    }
#endif
  else
    {
      return -EINVAL;
    }

#ifdef RZV2H_CANFD_HAVE_EXTID
  if (can_id & CAN_EFF_FLAG)
    {
      frame.id      = can_id & CAN_EFF_MASK;
      frame.id_mode = CAN_ID_MODE_EXTENDED;
    }
  else
#endif
    {
      frame.id      = can_id & CAN_SFF_MASK;
      frame.id_mode = CAN_ID_MODE_STANDARD;
    }

  frame.type = (can_id & CAN_RTR_FLAG) ? CAN_FRAME_TYPE_REMOTE
                                       : CAN_FRAME_TYPE_DATA;

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  /* Set the deadline before the frame is loaded, so the TX complete
   * interrupt of this frame always finds and clears it.
   */

  flags = enter_critical_section();
  priv->tx_deadline[mb] = deadline;
  leave_critical_section(flags);
#endif

  err = R_CANFD_Write(priv->hal_inst->p_ctrl, (uint32_t)mb, &frame);
  if (err != FSP_SUCCESS)
    {
      canerr("ERROR: R_CANFD_Write failed for CH%lu: %d\n",
             priv->hal_inst->p_cfg->channel, err);

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
      /* No frame was loaded, so drop its deadline */

      flags = enter_critical_section();
      priv->tx_deadline[mb].tv_sec  = 0;
      priv->tx_deadline[mb].tv_nsec = 0;
      leave_critical_section(flags);
#endif

      return -EIO;
    }

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  flags = enter_critical_section();
  rzv2h_canfd_sock_deadline_arm(priv);
  leave_critical_section(flags);
#endif

  /* The buffer is idle, so a packet still held for it was sent */

  if (priv->tx_pkt[mb] != NULL)
    {
      netpkt_free(dev, priv->tx_pkt[mb], NETPKT_TX);
    }

  priv->tx_pkt[mb] = pkt;
  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_receive
 *
 * Description:
 *   NuttX callback: return the oldest frame of the RX queue in a new
 *   packet, or NULL when the queue is empty. After a frame was dropped
 *   because the queue was full, also queue an RX overflow error frame.
 *
 * Input Parameters:
 *   dev - Network device of the channel
 *
 * Returned Value:
 *   Packet with one frame, or NULL
 *
 ****************************************************************************/

static netpkt_t *rzv2h_canfd_sock_receive(struct netdev_lowerhalf_s *dev)
{
  struct rzv2h_canfd_priv_s *priv =
    container_of(dev, struct rzv2h_canfd_priv_s, dev);
  struct rzv2h_canfd_sock_rx_s entry;
  irqstate_t flags;
  netpkt_t *pkt;
  bool lost;

  if (priv->rx_count == 0)
    {
      return NULL;
    }

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (pkt == NULL)
    {
      return NULL;
    }

  flags = enter_critical_section();

  if (priv->rx_count == 0)
    {
      leave_critical_section(flags);
      netpkt_free(dev, pkt, NETPKT_RX);
      return NULL;
    }

  entry          = priv->rx_queue[priv->rx_head];
  priv->rx_head  = (priv->rx_head + 1) % CONFIG_RZV2H_CANFD_SOCK_RX_DEPTH;
  priv->rx_count--;
  lost           = priv->rx_lost;
  priv->rx_lost  = false;

  leave_critical_section(flags);

  netpkt_copyin(dev, pkt, (const uint8_t *)&entry.frame,
                entry.fd ? sizeof(struct canfd_frame)
                         : sizeof(struct can_frame), 0);

#ifdef RZV2H_CANFD_HAVE_ERRORS
  if (lost)
    {
      rzv2h_canfd_report_rx_overflow(priv);
    }
#else
  UNUSED(lost);
#endif

  return pkt;
}

#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
/****************************************************************************
 * Name: rzv2h_canfd_sock_timing_rate
 *
 * Description:
 *   Bit rate and sample point of one FSP bit timing configuration.
 *
 * Input Parameters:
 *   t      - Bit timing
 *   rate   - Bit rate in bit/s, written on return
 *   sample - Sample point in percent, written on return
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rzv2h_canfd_sock_timing_rate(const can_bit_timing_cfg_t *t,
                                         uint32_t *rate, uint16_t *sample)
{
  uint32_t tq = 1 + t->time_segment_1 + t->time_segment_2;

  *rate   = RZV2H_CANFD_CLK_HZ / (t->baud_rate_prescaler * tq);
  *sample = (uint16_t)((100 * (1 + t->time_segment_1)) / tq);
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_get_bitrate
 *
 * Description:
 *   SIOCGCANBITRATE: report the nominal and data bit rates and sample
 *   points. With manual bit timing in Kconfig, compute them from the
 *   timing; otherwise report the values the next ifup uses.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   req  - Request, filled on return
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int rzv2h_canfd_sock_get_bitrate(struct rzv2h_canfd_priv_s *priv,
                                        struct can_ioctl_data_s *req)
{
  can_cfg_t const *cfg = priv->hal_inst->p_cfg;
  canfd_extended_cfg_t *ext = (canfd_extended_cfg_t *)cfg->p_extend;

  if (priv->use_manual_timing)
    {
      rzv2h_canfd_sock_timing_rate(cfg->p_bit_timing, &req->arbi_bitrate,
                                   &req->arbi_samplep);
      rzv2h_canfd_sock_timing_rate(ext->p_data_timing, &req->data_bitrate,
                                   &req->data_samplep);
    }
  else
    {
      req->arbi_bitrate = priv->baud;
      req->arbi_samplep = priv->sample_point;
      req->data_bitrate = priv->baud_data;
      req->data_samplep = priv->sample_point_data;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_canfd_sock_set_bitrate
 *
 * Description:
 *   SIOCSCANBITRATE: set the nominal and data bit rates and sample points
 *   of the automatic bit timing. net/netdev refuses this command while the
 *   interface is up, so the next ifup uses the new values. A field of 0
 *   keeps the current value. The data bit rate must not be lower than the
 *   nominal one.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   req  - Requested bit rates and sample points
 *
 * Returned Value:
 *   OK on success, -EINVAL if no bit timing fits a requested value
 *
 ****************************************************************************/

static int rzv2h_canfd_sock_set_bitrate(struct rzv2h_canfd_priv_s *priv,
                                        const struct can_ioctl_data_s *req)
{
  can_cfg_t const *cfg = priv->hal_inst->p_cfg;
  canfd_extended_cfg_t *ext = (canfd_extended_cfg_t *)cfg->p_extend;
  can_bit_timing_cfg_t timing;
  uint32_t baud;
  uint32_t baud_data;
  uint32_t sample;
  uint32_t sample_data;
  int ret;

  baud        = req->arbi_bitrate ? req->arbi_bitrate : priv->baud;
  sample      = req->arbi_samplep ? req->arbi_samplep : priv->sample_point;
  baud_data   = req->data_bitrate ? req->data_bitrate : priv->baud_data;
  sample_data = req->data_samplep ? req->data_samplep :
                                    priv->sample_point_data;

  if (baud_data < baud)
    {
      return -EINVAL;
    }

  ret = rzv2h_canfd_baud_to_timing(baud, sample, false,
                                   ext->delay_compensation, &timing);
  if (ret < 0)
    {
      return ret;
    }

  ret = rzv2h_canfd_baud_to_timing(baud_data, sample_data, true,
                                   ext->delay_compensation, &timing);
  if (ret < 0)
    {
      return ret;
    }

  priv->baud              = baud;
  priv->sample_point      = sample;
  priv->baud_data         = baud_data;
  priv->sample_point_data = sample_data;
  priv->use_manual_timing = false;
  return OK;
}
#endif /* CONFIG_NETDEV_CAN_BITRATE_IOCTL */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
/****************************************************************************
 * Name: rzv2h_canfd_sock_filter
 *
 * Description:
 *   SIOCACANSTDFILTER, SIOCACANEXTFILTER: add a filter
 *   (rzv2h_canfd_afl_add()). SIOCDCANSTDFILTER, SIOCDCANEXTFILTER: delete
 *   filter number 'fid1', as returned by the add command
 *   (rzv2h_canfd_afl_del()). This is the same as CANIOC_DEL_STDFILTER and
 *   CANIOC_DEL_EXTFILTER of the character driver.
 *
 * Input Parameters:
 *   priv - Channel private data
 *   cmd  - ioctl command
 *   arg  - Pointer to struct can_ioctl_filter_s
 *
 * Returned Value:
 *   Add: number of the new filter on success. Delete: OK on success.
 *   Negative errno on failure.
 *
 ****************************************************************************/

static int rzv2h_canfd_sock_filter(struct rzv2h_canfd_priv_s *priv, int cmd,
                                   const struct can_ioctl_filter_s *arg)
{
  struct rzv2h_canfd_filter_s req;
  bool ext = (cmd == SIOCACANEXTFILTER || cmd == SIOCDCANEXTFILTER);

  if (arg == NULL)
    {
      return -EINVAL;
    }

  if (cmd == SIOCDCANSTDFILTER || cmd == SIOCDCANEXTFILTER)
    {
      return rzv2h_canfd_afl_del(priv, arg->fid1, ext);
    }

  req.id1  = arg->fid1;
  req.id2  = arg->fid2;
  req.type = arg->ftype;
  req.prio = arg->fprio;
  req.ext  = ext;

  return rzv2h_canfd_afl_add(priv, &req);
}
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

#ifdef CONFIG_NETDEV_IOCTL
/****************************************************************************
 * Name: rzv2h_canfd_sock_ioctl
 *
 * Description:
 *   NuttX callback: handle the SocketCAN ioctl commands.
 *
 *   - SIOCGCANBITRATE / SIOCSCANBITRATE (CONFIG_NETDEV_CAN_BITRATE_IOCTL)
 *   - SIOCACANSTDFILTER / SIOCDCANSTDFILTER, SIOCACANEXTFILTER /
 *     SIOCDCANEXTFILTER and SIOCCANRECOVERY
 *     (CONFIG_NETDEV_CAN_FILTER_IOCTL)
 *
 *   SIOCACANSTDFILTER and SIOCACANEXTFILTER return the filter number.
 *   SIOCDCANSTDFILTER and SIOCDCANEXTFILTER take it in fid1, like
 *   CANIOC_DEL_STDFILTER.
 *
 * Input Parameters:
 *   dev - Network device of the channel
 *   cmd - ioctl command
 *   arg - Command argument
 *
 * Returned Value:
 *   OK on success (the filter number for SIOCACANSTDFILTER and
 *   SIOCACANEXTFILTER), negative errno on failure, -ENOTTY for an
 *   unsupported command
 *
 ****************************************************************************/

static int rzv2h_canfd_sock_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                                  unsigned long arg)
{
  struct rzv2h_canfd_priv_s *priv =
    container_of(dev, struct rzv2h_canfd_priv_s, dev);

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
      case SIOCGCANBITRATE:
        return rzv2h_canfd_sock_get_bitrate(priv,
                 (struct can_ioctl_data_s *)arg);

      case SIOCSCANBITRATE:
        return rzv2h_canfd_sock_set_bitrate(priv,
                 (const struct can_ioctl_data_s *)arg);
#endif

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
      case SIOCACANSTDFILTER:
      case SIOCDCANSTDFILTER:
#ifdef RZV2H_CANFD_HAVE_EXTID
      case SIOCACANEXTFILTER:
      case SIOCDCANEXTFILTER:
#endif
        return rzv2h_canfd_sock_filter(priv, cmd,
                 (const struct can_ioctl_filter_s *)arg);

      case SIOCCANRECOVERY:
        return rzv2h_canfd_ioctl_busoff_recovery(priv, 0);
#endif

      default:
        UNUSED(priv);
        return -ENOTTY;
    }
}
#endif /* CONFIG_NETDEV_IOCTL */
#else
/****************************************************************************
 * Name: rzv2h_canfd_ioctl
 *
 * Description:
 *   Handle ioctl commands for CAN configuration.
 *
 * Input Parameters:
 *   dev - CAN device
 *   cmd - ioctl command
 *   arg - Command argument
 *
 * Returned Value:
 *   Result of the command handler, -ENOTTY for an unsupported command
 *
 ****************************************************************************/

static int rzv2h_canfd_ioctl(struct can_dev_s *dev, int cmd,
                             unsigned long arg)
{
  struct rzv2h_canfd_priv_s *priv = dev->cd_priv;

  switch (cmd)
    {
      case CANIOC_SET_BITTIMING:
        return rzv2h_canfd_ioctl_set_bittiming(priv, arg);
      case CANIOC_GET_BITTIMING:
        return rzv2h_canfd_ioctl_get_bittiming(priv, arg);
      case CANIOC_SET_CONNMODES:
        return rzv2h_canfd_ioctl_set_connmodes(priv, arg);
      case CANIOC_GET_CONNMODES:
        return rzv2h_canfd_ioctl_get_connmodes(priv, arg);
      case CANIOC_IFLUSH:
        return rzv2h_canfd_ioctl_iflush(priv, arg);
      case CANIOC_OFLUSH:
        return rzv2h_canfd_ioctl_oflush(priv, arg);
      case CANIOC_IOFLUSH:
        return rzv2h_canfd_ioctl_ioflush(priv, arg);
      case CANIOC_ADD_STDFILTER:
        return rzv2h_canfd_ioctl_add_stdfilter(priv, arg);
      case CANIOC_DEL_STDFILTER:
        return rzv2h_canfd_ioctl_del_stdfilter(priv, arg);
#ifdef CONFIG_CAN_EXTID
      case CANIOC_ADD_EXTFILTER:
        return rzv2h_canfd_ioctl_add_extfilter(priv, arg);
      case CANIOC_DEL_EXTFILTER:
        return rzv2h_canfd_ioctl_del_extfilter(priv, arg);
#endif
      case CANIOC_BUSOFF_RECOVERY:
        return rzv2h_canfd_ioctl_busoff_recovery(priv, arg);
      default:
        caninfo("CANFD%lu: Unsupported ioctl %d\n",
                priv->hal_inst->p_cfg->channel, cmd);
        return -ENOTTY;
    }
}
#endif /* CONFIG_RZV2H_CANFD_SOCKET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RZV2H_CANFD_SOCKET
/****************************************************************************
 * Name: rzv2h_canfd_sock_initialize
 *
 * Description:
 *   Register one CANFD channel as the SocketCAN network interface
 *   can<channel>. The hardware is set up later, in
 *   rzv2h_canfd_sock_ifup(), when the interface is brought up.
 *
 * Input Parameters:
 *   channel - CANFD channel number (0-5)
 *
 * Returned Value:
 *   OK on success, -ENODEV if the channel is not enabled, or the negative
 *   errno of netdev_lower_register()
 *
 ****************************************************************************/

int rzv2h_canfd_sock_initialize(int channel)
{
  struct rzv2h_canfd_priv_s *priv = NULL;
  struct netdev_lowerhalf_s *dev;
  int ret;

  if (channel >= 0)
    {
      priv = rzv2h_canfd_ch_priv((uint32_t)channel);
    }

  if (priv == NULL)
    {
      canerr("ERROR: Invalid CANFD channel %d\n", channel);
      return -ENODEV;
    }

  priv->sample_point_data = priv->sample_point;

  /* At most txmb_num frames in the TX message buffers and the RX queue
   * full.
   */

  dev = &priv->dev;
  dev->ops = &g_rzv2h_canfd_sock_ops;
  dev->quota[NETPKT_TX] = priv->txmb_num;
  dev->quota[NETPKT_RX] = CONFIG_RZV2H_CANFD_SOCK_RX_DEPTH;

  /* Name the interface after the channel. netdev_register() uses a name
   * set here instead of the next free can<n> number.
   */

  snprintf(dev->netdev.d_ifname, IFNAMSIZ, "can%d", channel);

  ret = netdev_lower_register(dev, NET_LL_CAN);
  if (ret < 0)
    {
      canerr("ERROR: CANFD%d: netdev_lower_register failed: %d\n",
             channel, ret);
      return ret;
    }

  caninfo("CANFD%d: Registered as %s\n", channel, dev->netdev.d_ifname);
  return OK;
}
#else
/****************************************************************************
 * Name: rzv2h_canfd_initialize
 *
 * Description:
 *   Return the NuttX CAN device of one CANFD channel. The hardware is set
 *   up later, in rzv2h_canfd_setup(), when the device is first opened.
 *
 * Input Parameters:
 *   channel - CANFD channel number (0-5)
 *
 * Returned Value:
 *   CAN device instance on success, NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *rzv2h_canfd_initialize(int channel)
{
  struct rzv2h_canfd_priv_s *priv = NULL;

  /* Select the device for the given channel */

  if (channel >= 0)
    {
      priv = rzv2h_canfd_ch_priv((uint32_t)channel);
    }

  if (priv == NULL)
    {
      canerr("ERROR: Invalid CANFD channel %d\n", channel);
      return NULL;
    }

  caninfo("CANFD%d: Initialized\n", channel);
  return &priv->dev;
}
#endif /* CONFIG_RZV2H_CANFD_SOCKET */

#endif /* CONFIG_RZV2H_CANFD0 || ... || CONFIG_RZV2H_CANFD5 */
