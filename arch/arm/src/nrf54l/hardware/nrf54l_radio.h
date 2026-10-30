/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_radio.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_RADIO_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_RADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#undef  HAVE_RADIO_NRF250KBIT
#define HAVE_RADIO_BLELR
#define HAVE_RADIO_IEEE802154

/* Register offsets *********************************************************/

#define NRF54L_RADIO_TASKS_TXEN_OFFSET       0x0000                 /* Enable RADIO in TX mode */
#define NRF54L_RADIO_TASKS_RXEN_OFFSET       0x0004                 /* Enable RADIO in RX mode */
#define NRF54L_RADIO_TASKS_START_OFFSET      0x0008                 /* Start RADIO */
#define NRF54L_RADIO_TASKS_STOP_OFFSET       0x000c                 /* Stop RADIO */
#define NRF54L_RADIO_TASKS_DISABLE_OFFSET    0x0010                 /* Disable RADIO */
#define NRF54L_RADIO_TASKS_RSSISTART_OFFSET  0x0014                 /* Start the RSSI measurement */
#define NRF54L_RADIO_TASKS_BCSTART_OFFSET    0x0018                 /* Start the bit counter */
#define NRF54L_RADIO_TASKS_BCSTOP_OFFSET     0x001c                 /* Stop the bit counter */
#define NRF54L_RADIO_TASKS_EDSTART_OFFSET    0x0020                 /* Start the energy detect measurement used in IEEE 802.15.4 mode */
#define NRF54L_RADIO_TASKS_EDSTOP_OFFSET     0x0024                 /* Stop the energy detect measurement */
#define NRF54L_RADIO_TASKS_CCASTART_OFFSET   0x0028                 /* Start the clear channel assessment used in IEEE 802.15.4 mode */
#define NRF54L_RADIO_TASKS_CCASTOP_OFFSET    0x002c                 /* Stop the clear channel assessment */
#define NRF54L_RADIO_TASKS_SOFTRESET_OFFSET  0x00a4                 /* Reset all public registers */
#define NRF54L_RADIO_SUBSCRIBE_TXEN_OFFSET   0x0100                 /* Subscribe configuration for task TXEN */
#define NRF54L_RADIO_SUBSCRIBE_RXEN_OFFSET   0x0104                 /* Subscribe configuration for task RXEN */
#define NRF54L_RADIO_SUBSCRIBE_DISABLE_OFFSET 0x0110                /* Subscribe configuration for task DISABLE */
#define NRF54L_RADIO_EVENTS_READY_OFFSET     0x0200                 /* RADIO has ramped up and is ready to be started */
#define NRF54L_RADIO_EVENTS_TXREADY_OFFSET   0x0204                 /* RADIO has ramped up and is ready to be started TX path */
#define NRF54L_RADIO_EVENTS_RXREADY_OFFSET   0x0208                 /* RADIO has ramped up and is ready to be started RX path */
#define NRF54L_RADIO_EVENTS_ADDRESS_OFFSET   0x020c                 /* Address sent or received */
#define NRF54L_RADIO_EVENTS_FRAMESTART_OFFSET0x0210                 /* IEEE 802.15.4 length field received */
#define NRF54L_RADIO_EVENTS_PAYLOAD_OFFSET   0x0214                 /* Packet payload sent or received */
#define NRF54L_RADIO_EVENTS_END_OFFSET       0x0218                 /* Packet sent or received */
#define NRF54L_RADIO_EVENTS_PHYEND_OFFSET    0x021c                 /* Last bit is sent on air */
#define NRF54L_RADIO_EVENTS_DISABLED_OFFSET  0x0220                 /* RADIO has been disabled */
#define NRF54L_RADIO_EVENTS_DEVMATCH_OFFSET  0x0224                 /* A device address match occurred on the last received packet */
#define NRF54L_RADIO_EVENTS_DEVMISS_OFFSET   0x0228                 /* No device address match occurred on the last received packet */
#define NRF54L_RADIO_EVENTS_CRCOK_OFFSET     0x022c                 /* Packet received with CRC ok */
#define NRF54L_RADIO_EVENTS_CRCERROR_OFFSET  0x0230                 /* Packet received with CRC error */
#define NRF54L_RADIO_EVENTS_BCMATCH_OFFSET   0x0238                 /* Bit counter reached bit count value */
#define NRF54L_RADIO_EVENTS_EDEND_OFFSET     0x023c                 /* Sampling of energy detection complete */
#define NRF54L_RADIO_EVENTS_EDSTOPPED_OFFSET 0x0240                 /* The sampling of energy detection has stopped */
#define NRF54L_RADIO_EVENTS_CCAIDLE_OFFSET   0x0244                 /* Wireless medium in idle - clear to send */
#define NRF54L_RADIO_EVENTS_CCABUSY_OFFSET   0x0248                 /* Wireless medium busy - do not send */
#define NRF54L_RADIO_EVENTS_CCASTOPPED_OFFSET0x024c                 /* The CCA has stopped */
#define NRF54L_RADIO_EVENTS_RATEBOOST_OFFSET 0x0250                 /* Ble_LR CI field received */
#define NRF54L_RADIO_EVENTS_MHRMATCH_OFFSET  0x0254                 /* MAC header match found */
#define NRF54L_RADIO_EVENTS_SYNC_OFFSET      0x0258                 /* Possible preamble received */
#define NRF54L_RADIO_PUBLISH_READY_OFFSET    0x0300                 /* Publish configuration for event READY */
#define NRF54L_RADIO_PUBLISH_ADDRESS_OFFSET  0x030c                 /* Publish configuration for event ADDRESS */
#define NRF54L_RADIO_PUBLISH_PAYLOAD_OFFSET  0x0314                 /* Publish configuration for event PAYLOAD */
#define NRF54L_RADIO_PUBLISH_END_OFFSET      0x0318                 /* Publish configuration for event END */
#define NRF54L_RADIO_PUBLISH_PHYEND_OFFSET   0x031c                 /* Publish configuration for event PHYEND */
#define NRF54L_RADIO_PUBLISH_DISABLED_OFFSET 0x0320                 /* Publish configuration for event DISABLED */
#define NRF54L_RADIO_PUBLISH_BCMATCH_OFFSET  0x0338                 /* Publish configuration for event BCMATCH */
#define NRF54L_RADIO_SHORTS_OFFSET           0x0400                 /* Shortcuts between local events and tasks */
#define NRF54L_RADIO_INTENSET00_OFFSET       0x0488                 /* Enable interrupt */
#define NRF54L_RADIO_INTENCLR00_OFFSET       0x0490                 /* Disable interrupt */
#define NRF54L_RADIO_MODE_OFFSET             0x0500                 /* Data rate and modulation */
#define NRF54L_RADIO_STATE_OFFSET            0x0520                 /* Current radio state */
#define NRF54L_RADIO_EDCTRL_OFFSET           0x0530                 /* IEEE 802.15.4 energy detect control */
#define NRF54L_RADIO_EDSAMPLE_OFFSET         0x0534                 /* IEEE 802.15.4 energy detect level */
#define NRF54L_RADIO_CCACTRL_OFFSET          0x0538                 /* IEEE 802.15.4 clear channel assessment control */
#define NRF54L_RADIO_DATAWHITE_OFFSET        0x0540                 /* Data whitening configuration */
#define NRF54L_RADIO_TIMING_OFFSET           0x0704                 /* Timing */
#define NRF54L_RADIO_FREQUENCY_OFFSET        0x0708                 /* Frequency */
#define NRF54L_RADIO_TXPOWER_OFFSET          0x0710                 /* Output power */
#define NRF54L_RADIO_TIFS_OFFSET             0x0714                 /* Interframe spacing in us */
#define NRF54L_RADIO_RSSISAMPLE_OFFSET       0x0718                 /* RSSI sample */
#define NRF54L_RADIO_CRCSTATUS_OFFSET        0x0e0c                 /* CRC status */
#define NRF54L_RADIO_RXMATCH_OFFSET          0x0e10                 /* Received address */
#define NRF54L_RADIO_RXCRC_OFFSET            0x0e14                 /* CRC field of previously received packet */
#define NRF54L_RADIO_DAI_OFFSET              0x0e18                 /* Device address match index */
#define NRF54L_RADIO_PDUSTAT_OFFSET          0x0e1c                 /* Payload status */
#define NRF54L_RADIO_PCNF0_OFFSET            0x0e20                 /* Packet configuration register 0 */
#define NRF54L_RADIO_PCNF1_OFFSET            0x0e28                 /* Packet configuration register 1 */
#define NRF54L_RADIO_BASE0_OFFSET            0x0e2c                 /* Base address 0 */
#define NRF54L_RADIO_BASE1_OFFSET            0x0e30                 /* Base address 1 */
#define NRF54L_RADIO_PREFIX0_OFFSET          0x0e34                 /* Prefixes bytes for logical addresses 0-3 */
#define NRF54L_RADIO_PREFIX1_OFFSET          0x0e38                 /* Prefixes bytes for logical addresses 4-7 */
#define NRF54L_RADIO_TXADDRESS_OFFSET        0x0e3c                 /* Transmit address select */
#define NRF54L_RADIO_RXADDRESSES_OFFSET      0x0e40                 /* Receive address select */
#define NRF54L_RADIO_CRCCNF_OFFSET           0x0e44                 /* CRC configuration */
#define NRF54L_RADIO_CRCPOLY_OFFSET          0x0e48                 /* CRC polynomial */
#define NRF54L_RADIO_CRCINIT_OFFSET          0x0e4c                 /* CRC initial value */
#define NRF54L_RADIO_DAB_OFFSET(p)           (0x0e50 + ((p) * 0x4)) /* Device address base segment */
#define NRF54L_RADIO_DAP_OFFSET(p)           (0x0e70 + ((p) * 0x4)) /* Device address prefix */
#define NRF54L_RADIO_DACNF_OFFSET            0x0e90                 /* Device address match configuration */
#define NRF54L_RADIO_BCC_OFFSET              0x0e94                 /* Bit counter compare */
#define NRF54L_RADIO_MHRMATCHCONF_OFFSET     0x0eb4                 /* Search pattern configuration */
#define NRF54L_RADIO_MHRMATCHMASK_OFFSET     0x0eb8                 /* Pattern mask */
#define NRF54L_RADIO_SFD_OFFSET              0x0ebc                 /* IEEE 802.15.4 start of frame delimiter */
#define NRF54L_RADIO_PACKETPTR_OFFSET        0x0ed0                 /* Packet pointer */

/* Register Addresses *******************************************************/

#define NRF54L_RADIO_TASKS_TXEN          (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_TXEN_OFFSET)
#define NRF54L_RADIO_TASKS_RXEN          (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_RXEN_OFFSET)
#define NRF54L_RADIO_TASKS_START         (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_START_OFFSET)
#define NRF54L_RADIO_TASKS_STOP          (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_STOP_OFFSET)
#define NRF54L_RADIO_TASKS_DISABLE       (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_DISABLE_OFFSET)
#define NRF54L_RADIO_TASKS_RSSISTART     (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_RSSISTART_OFFSET)
#define NRF54L_RADIO_TASKS_BCSTART       (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_BCSTART_OFFSET)
#define NRF54L_RADIO_TASKS_BCSTOP        (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_BCSTOP_OFFSET)
#define NRF54L_RADIO_TASKS_EDSTART       (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_EDSTART_OFFSET)
#define NRF54L_RADIO_TASKS_EDSTOP        (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_EDSTOP_OFFSET)
#define NRF54L_RADIO_TASKS_CCASTART      (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_CCASTART_OFFSET)
#define NRF54L_RADIO_TASKS_CCASTOP       (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_CCASTOP_OFFSET)
#define NRF54L_RADIO_TASKS_SOFTRESET     (NRF54L_RADIO_BASE + NRF54L_RADIO_TASKS_SOFTRESET_OFFSET)
#define NRF54L_RADIO_SUBSCRIBE_TXEN      (NRF54L_RADIO_BASE + NRF54L_RADIO_SUBSCRIBE_TXEN_OFFSET)
#define NRF54L_RADIO_SUBSCRIBE_RXEN      (NRF54L_RADIO_BASE + NRF54L_RADIO_SUBSCRIBE_RXEN_OFFSET)
#define NRF54L_RADIO_SUBSCRIBE_DISABLE   (NRF54L_RADIO_BASE + NRF54L_RADIO_SUBSCRIBE_DISABLE_OFFSET)
#define NRF54L_RADIO_EVENTS_READY        (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_READY_OFFSET)
#define NRF54L_RADIO_EVENTS_TXREADY      (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_TXREADY_OFFSET)
#define NRF54L_RADIO_EVENTS_RXREADY      (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_RXREADY_OFFSET)
#define NRF54L_RADIO_EVENTS_ADDRESS      (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_ADDRESS_OFFSET)
#define NRF54L_RADIO_EVENTS_FRAMESTART   (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_FRAMESTART_OFFSET)
#define NRF54L_RADIO_EVENTS_PAYLOAD      (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_PAYLOAD_OFFSET)
#define NRF54L_RADIO_EVENTS_END          (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_END_OFFSET)
#define NRF54L_RADIO_EVENTS_PHYEND       (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_PHYEND_OFFSET)
#define NRF54L_RADIO_EVENTS_DISABLED     (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_DISABLED_OFFSET)
#define NRF54L_RADIO_EVENTS_DEVMATCH     (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_DEVMATCH_OFFSET)
#define NRF54L_RADIO_EVENTS_DEVMISS      (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_DEVMISS_OFFSET)
#define NRF54L_RADIO_EVENTS_CRCOK        (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_CRCOK_OFFSET)
#define NRF54L_RADIO_EVENTS_CRCERROR     (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_CRCERROR_OFFSET)
#define NRF54L_RADIO_EVENTS_BCMATCH      (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_BCMATCH_OFFSET)
#define NRF54L_RADIO_EVENTS_EDEND        (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_EDEND_OFFSET)
#define NRF54L_RADIO_EVENTS_EDSTOPPED    (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_EDSTOPPED_OFFSET)
#define NRF54L_RADIO_EVENTS_CCAIDLE      (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_CCAIDLE_OFFSET)
#define NRF54L_RADIO_EVENTS_CCABUSY      (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_CCABUSY_OFFSET)
#define NRF54L_RADIO_EVENTS_CCASTOPPED   (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_CCASTOPPED_OFFSET)
#define NRF54L_RADIO_EVENTS_RATEBOOST    (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_RATEBOOST_OFFSET)
#define NRF54L_RADIO_EVENTS_MHRMATCH     (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_MHRMATCH_OFFSET)
#define NRF54L_RADIO_EVENTS_SYNC         (NRF54L_RADIO_BASE + NRF54L_RADIO_EVENTS_SYNC_OFFSET)
#define NRF54L_RADIO_PUBLISH_READY       (NRF54L_RADIO_BASE + NRF54L_RADIO_PUBLISH_READY_OFFSET)
#define NRF54L_RADIO_PUBLISH_ADDRESS     (NRF54L_RADIO_BASE + NRF54L_RADIO_PUBLISH_ADDRESS_OFFSET)
#define NRF54L_RADIO_PUBLISH_PAYLOAD     (NRF54L_RADIO_BASE + NRF54L_RADIO_PUBLISH_PAYLOAD_OFFSET)
#define NRF54L_RADIO_PUBLISH_END         (NRF54L_RADIO_BASE + NRF54L_RADIO_PUBLISH_END_OFFSET)
#define NRF54L_RADIO_PUBLISH_PHYEND      (NRF54L_RADIO_BASE + NRF54L_RADIO_PUBLISH_PHYEND_OFFSET)
#define NRF54L_RADIO_PUBLISH_DISABLED    (NRF54L_RADIO_BASE + NRF54L_RADIO_PUBLISH_DISABLED_OFFSET)
#define NRF54L_RADIO_PUBLISH_BCMATCH     (NRF54L_RADIO_BASE + NRF54L_RADIO_PUBLISH_BCMATCH_OFFSET)
#define NRF54L_RADIO_SHORTS              (NRF54L_RADIO_BASE + NRF54L_RADIO_SHORTS_OFFSET)
#define NRF54L_RADIO_INTENSET00          (NRF54L_RADIO_BASE + NRF54L_RADIO_INTENSET00_OFFSET)
#define NRF54L_RADIO_INTENCLR00          (NRF54L_RADIO_BASE + NRF54L_RADIO_INTENCLR00_OFFSET)
#define NRF54L_RADIO_MODE                (NRF54L_RADIO_BASE + NRF54L_RADIO_MODE_OFFSET)
#define NRF54L_RADIO_STATE               (NRF54L_RADIO_BASE + NRF54L_RADIO_STATE_OFFSET)
#define NRF54L_RADIO_EDCTRL              (NRF54L_RADIO_BASE + NRF54L_RADIO_EDCTRL_OFFSET)
#define NRF54L_RADIO_EDSAMPLE            (NRF54L_RADIO_BASE + NRF54L_RADIO_EDSAMPLE_OFFSET)
#define NRF54L_RADIO_CCACTRL             (NRF54L_RADIO_BASE + NRF54L_RADIO_CCACTRL_OFFSET)
#define NRF54L_RADIO_DATAWHITE           (NRF54L_RADIO_BASE + NRF54L_RADIO_DATAWHITE_OFFSET)
#define NRF54L_RADIO_TIMING              (NRF54L_RADIO_BASE + NRF54L_RADIO_TIMING_OFFSET)
#define NRF54L_RADIO_FREQUENCY           (NRF54L_RADIO_BASE + NRF54L_RADIO_FREQUENCY_OFFSET)
#define NRF54L_RADIO_TXPOWER             (NRF54L_RADIO_BASE + NRF54L_RADIO_TXPOWER_OFFSET)
#define NRF54L_RADIO_TIFS                (NRF54L_RADIO_BASE + NRF54L_RADIO_TIFS_OFFSET)
#define NRF54L_RADIO_RSSISAMPLE          (NRF54L_RADIO_BASE + NRF54L_RADIO_RSSISAMPLE_OFFSET)
#define NRF54L_RADIO_CRCSTATUS           (NRF54L_RADIO_BASE + NRF54L_RADIO_CRCSTATUS_OFFSET)
#define NRF54L_RADIO_RXMATCH             (NRF54L_RADIO_BASE + NRF54L_RADIO_RXMATCH_OFFSET)
#define NRF54L_RADIO_RXCRC               (NRF54L_RADIO_BASE + NRF54L_RADIO_RXCRC_OFFSET)
#define NRF54L_RADIO_DAI                 (NRF54L_RADIO_BASE + NRF54L_RADIO_DAI_OFFSET)
#define NRF54L_RADIO_PDUSTAT             (NRF54L_RADIO_BASE + NRF54L_RADIO_PDUSTAT_OFFSET)
#define NRF54L_RADIO_PCNF0               (NRF54L_RADIO_BASE + NRF54L_RADIO_PCNF0_OFFSET)
#define NRF54L_RADIO_PCNF1               (NRF54L_RADIO_BASE + NRF54L_RADIO_PCNF1_OFFSET)
#define NRF54L_RADIO_BASE0               (NRF54L_RADIO_BASE + NRF54L_RADIO_BASE0_OFFSET)
#define NRF54L_RADIO_BASE1               (NRF54L_RADIO_BASE + NRF54L_RADIO_BASE1_OFFSET)
#define NRF54L_RADIO_PREFIX0             (NRF54L_RADIO_BASE + NRF54L_RADIO_PREFIX0_OFFSET)
#define NRF54L_RADIO_PREFIX1             (NRF54L_RADIO_BASE + NRF54L_RADIO_PREFIX1_OFFSET)
#define NRF54L_RADIO_TXADDRESS           (NRF54L_RADIO_BASE + NRF54L_RADIO_TXADDRESS_OFFSET)
#define NRF54L_RADIO_RXADDRESSES         (NRF54L_RADIO_BASE + NRF54L_RADIO_RXADDRESSES_OFFSET)
#define NRF54L_RADIO_CRCCNF              (NRF54L_RADIO_BASE + NRF54L_RADIO_CRCCNF_OFFSET)
#define NRF54L_RADIO_CRCPOLY             (NRF54L_RADIO_BASE + NRF54L_RADIO_CRCPOLY_OFFSET)
#define NRF54L_RADIO_CRCINIT             (NRF54L_RADIO_BASE + NRF54L_RADIO_CRCINIT_OFFSET)
#define NRF54L_RADIO_DAB(p)              (NRF54L_RADIO_BASE + NRF54L_RADIO_DAB_OFFSET(p))
#define NRF54L_RADIO_DAP(p)              (NRF54L_RADIO_BASE + NRF54L_RADIO_DAP_OFFSET(p))
#define NRF54L_RADIO_DACNF               (NRF54L_RADIO_BASE + NRF54L_RADIO_DACNF_OFFSET)
#define NRF54L_RADIO_BCC                 (NRF54L_RADIO_BASE + NRF54L_RADIO_BCC_OFFSET)
#define NRF54L_RADIO_MHRMATCHCONF        (NRF54L_RADIO_BASE + NRF54L_RADIO_MHRMATCHCONF_OFFSET)
#define NRF54L_RADIO_MHRMATCHMASK        (NRF54L_RADIO_BASE + NRF54L_RADIO_MHRMATCHMASK_OFFSET)
#define NRF54L_RADIO_SFD                 (NRF54L_RADIO_BASE + NRF54L_RADIO_SFD_OFFSET)
#define NRF54L_RADIO_PACKETPTR           (NRF54L_RADIO_BASE + NRF54L_RADIO_PACKETPTR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SUBSCRIBE and PUBLISH Registers */

#define RADIO_SUBSCRIBE_CHIDX_SHIFT     (0)         /* Bits 0-7: DPPI channel */
#define RADIO_SUBSCRIBE_CHIDX_MASK      (0xff << RADIO_SUBSCRIBE_CHIDX_SHIFT)
#define RADIO_SUBSCRIBE_EN              (1 << 31)   /* Bit 31: Enable subscription */
#define RADIO_PUBLISH_CHIDX_SHIFT       (0)         /* Bits 0-7: DPPI channel */
#define RADIO_PUBLISH_CHIDX_MASK        (0xff << RADIO_PUBLISH_CHIDX_SHIFT)
#define RADIO_PUBLISH_EN                (1 << 31)   /* Bit 31: Enable publishing */

/* SHORTS Register */

#define RADIO_SHORTS_READY_START        (1 << 0)    /* Bit 0: Shortcut between event READY and task START */
#define RADIO_SHORTS_DISABLED_TXEN      (1 << 2)    /* Bit 2: Shortcut between event DISABLED and task TXEN */
#define RADIO_SHORTS_DISABLED_RXEN      (1 << 3)    /* Bit 3: Shortcut between event DISABLED and task RXEN */
#define RADIO_SHORTS_ADDRESS_RSSISTART  (1 << 4)    /* Bit 4: Shortcut between event ADDRESS and task RSSISTART */
#define RADIO_SHORTS_END_START          (1 << 5)    /* Bit 5: Shortcut between event END and task START */
#define RADIO_SHORTS_ADDRESS_BCSTART    (1 << 6)    /* Bit 6: Shortcut between event ADDRESS and task BCSTART */
#define RADIO_SHORTS_PHYEND_PLLEN       (1 << 7)    /* Bit 7: Shortcut between event PHYEND and task PLLEN */
#define RADIO_SHORTS_RXREADY_CCASTART   (1 << 10)   /* Bit 10: Shortcut between event RXREADY and task CCASTART */
#define RADIO_SHORTS_CCAIDLE_TXEN       (1 << 11)   /* Bit 11: Shortcut between event CCAIDLE and task TXEN */
#define RADIO_SHORTS_CCABUSY_DISABLE    (1 << 12)   /* Bit 12: Shortcut between event CCABUSY and task DISABLE */
#define RADIO_SHORTS_FRAMESTART_BCSTART (1 << 13)   /* Bit 13: Shortcut between event FRAMESTART and task BCSTART */
#define RADIO_SHORTS_READY_EDSTART      (1 << 14)   /* Bit 14: Shortcut between event READY and task EDSTART */
#define RADIO_SHORTS_EDEND_DISABLE      (1 << 15)   /* Bit 15: Shortcut between event EDEND and task DISABLE */
#define RADIO_SHORTS_CCAIDLE_STOP       (1 << 16)   /* Bit 16: Shortcut between event CCAIDLE and task STOP */
#define RADIO_SHORTS_TXREADY_START      (1 << 17)   /* Bit 17: Shortcut between event TXREADY and task START */
#define RADIO_SHORTS_RXREADY_START      (1 << 18)   /* Bit 18: Shortcut between event RXREADY and task START */
#define RADIO_SHORTS_PHYEND_DISABLE     (1 << 19)   /* Bit 19: Shortcut between event PHYEND and task DISABLE */
#define RADIO_SHORTS_PHYEND_START       (1 << 20)   /* Bit 20: Shortcut between event PHYEND and task START */
#define RADIO_SHORTS_PLLREADY_TXEN      (1 << 27)   /* Bit 27: Shortcut between event PLLREADY and task TXEN */
#define RADIO_SHORTS_PLLREADY_RXEN      (1 << 28)   /* Bit 28: Shortcut between event PLLREADY and task RXEN */

/* INTENSET00/INTENCLR00 Register */

#define RADIO_INT_READY                 (1 << 0)    /* Bit 0: Interrupt for event READY */
#define RADIO_INT_TXREADY               (1 << 1)    /* Bit 1: Interrupt for event TXREADY */
#define RADIO_INT_RXREADY               (1 << 2)    /* Bit 2: Interrupt for event RXREADY */
#define RADIO_INT_ADDRESS               (1 << 3)    /* Bit 3: Interrupt for event ADDRESS */
#define RADIO_INT_FRAMESTART            (1 << 4)    /* Bit 4: Interrupt for event FRAMESTART */
#define RADIO_INT_PAYLOAD               (1 << 5)    /* Bit 5: Interrupt for event PAYLOAD */
#define RADIO_INT_END                   (1 << 6)    /* Bit 6: Interrupt for event END */
#define RADIO_INT_PHYEND                (1 << 7)    /* Bit 7: Interrupt for event PHYEND */
#define RADIO_INT_DISABLED              (1 << 8)    /* Bit 8: Interrupt for event DISABLED */
#define RADIO_INT_DEVMATCH              (1 << 9)    /* Bit 9: Interrupt for event DEVMATCH */
#define RADIO_INT_DEVMISS               (1 << 10)   /* Bit 10: Interrupt for event DEVMISS */
#define RADIO_INT_CRCOK                 (1 << 11)   /* Bit 11: Interrupt for event CRCOK */
#define RADIO_INT_CRCERROR              (1 << 12)   /* Bit 12: Interrupt for event CRCERROR */
#define RADIO_INT_BCMATCH               (1 << 14)   /* Bit 14: Interrupt for event BCMATCH */
#define RADIO_INT_EDEND                 (1 << 15)   /* Bit 15: Interrupt for event EDEND */
#define RADIO_INT_EDSTOPPED             (1 << 16)   /* Bit 16: Interrupt for event EDSTOPPED */
#define RADIO_INT_CCAIDLE               (1 << 17)   /* Bit 17: Interrupt for event CCAIDLE */
#define RADIO_INT_CCABUSY               (1 << 18)   /* Bit 18: Interrupt for event CCABUSY */
#define RADIO_INT_CCASTOPPED            (1 << 19)   /* Bit 19: Interrupt for event CCASTOPPED */
#define RADIO_INT_RATEBOOST             (1 << 20)   /* Bit 20: Interrupt for event RATEBOOST */
#define RADIO_INT_MHRMATCH              (1 << 21)   /* Bit 21: Interrupt for event MHRMATCH */
#define RADIO_INT_SYNC                  (1 << 22)   /* Bit 22: Interrupt for event SYNC */

/* CRCSTATUS Register */

#define RADIO_CRCSTATUS_OK              (1 << 0)    /* Bit 0: CRC status of packet received */

/* RXMATCH Register */

#define RADIO_RXMATCH_SHIFT             (0)         /* Bits 0-2: Received address */
#define RADIO_RXMATCH_MASK              (0x7 << RADIO_RXMATCH_SHIFT)

/* RXCRC Register */

#define RADIO_RXCRC_SHIFT               (0)         /* Bits 0-23: CRC Field of previously received packet */
#define RADIO_RXCRC_MASK                (0x00ffffff << RADIO_RXCRC_SHIFT)

/* DAI Register */

#define RADIO_DAI_SHIFT                 (0)         /* Bits 0-2: Device address match index */
#define RADIO_DAI_MASK                  (0x7 << RADIO_DAI_SHIFT)

/* PDUSTAT Register */

#define RADIO_PDUSTAT_PDUSTAT             (1 << 0)  /* Bit 0: Status on payload length vs. PCNF1.MAXLEN */
#define RADIO_PDUSTAT_CISTAT_SHIFT        (1)       /* Bits 1-2: Status on what rate packet is received with in Long Range */
#define RADIO_PDUSTAT_CISTAT_MASK         (0x3 << RADIO_PDUSTAT_CISTAT_SHIFT)
#  define RADIO_PDUSTAT_CISTAT_LR125KBIT  (0 << RADIO_PDUSTAT_CISTAT_SHIFT) /* 0: Frame is received at 125kbps */
#  define RADIO_PDUSTAT_CISTAT_LR500KBIT  (1 << RADIO_PDUSTAT_CISTAT_SHIFT) /* 1: Frame is received at 500kbps */

/* FREQUENCY Register */

#define RADIO_FREQUENCY_SHIFT             (0)       /* Bits 0-6: Radio channel frequency*/
#define RADIO_FREQUENCY_MASK              (0x7f << RADIO_FREQUENCY_SHIFT)
#define RADIO_FREQUENCY_MAP_2400MHZ       (0 << 8)  /* Bit 8: Channel map between 2400 MHz - 2500 MHz*/
#define RADIO_FREQUENCY_MAP_2360MHZ       (1 << 8)  /* Bit 8: Channel map between 2360 MHz - 2460 MHz*/

/* TXPOWER Register */

#define RADIO_TXPOWER_SHIFT               (0)      /* Bits 0-10: RADIO output power */
#define RADIO_TXPOWER_MASK                (0x7ff << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG46DBM          (0x110 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG40DBM          (0x130 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG28DBM          (0x001 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG20DBM          (0x002 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG18DBM          (0x003 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG16DBM          (0x004 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG14DBM          (0x005 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG12DBM          (0x006 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG10DBM          (0x007 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG9DBM           (0x008 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG8DBM           (0x009 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG7DBM           (0x00a << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG6DBM           (0x00b << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG5DBM           (0x00d << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG4DBM           (0x00f << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG3DBM           (0x011 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG2DBM           (0x013 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_NEG1DBM           (0x015 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_0DBM              (0x018 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_POS1DBM           (0x01b << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_POS2DBM           (0x01f << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_POS3DBM           (0x023 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_POS4DBM           (0x028 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_POS5DBM           (0x02d << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_POS6DBM           (0x033 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_POS7DBM           (0x039 << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_POS8DBM           (0x03f << RADIO_TXPOWER_SHIFT)
#  define RADIO_TXPOWER_MAXDBM            (0x03f << RADIO_TXPOWER_SHIFT)

/* MODE Register */

#define RADIO_MODE_SHIFT                  (0)                        /* Bits 0-3: Radio data rate and modulation setting (FSK) */
#define RADIO_MODE_MASK                   (0xf << RADIO_MODE_SHIFT)
#define RADIO_MODE_NRF1MBIT               (0x00 << RADIO_MODE_SHIFT) /* 0: 1 Mbit/s Nordic proprietary radio mode */
#define RADIO_MODE_NRF2MBIT               (0x01 << RADIO_MODE_SHIFT) /* 1: 2 Mbit/s Nordic proprietary radio mode */
#ifdef HAVE_RADIO_NRF250KBIT
#  define RADIO_MODE_NRF250KBIT           (0x02 << RADIO_MODE_SHIFT) /* 2: 250 kbit/s Nordic proprietary radio mode (deprecated) */
#endif
#define RADIO_MODE_BLE1MBIT               (0x03 << RADIO_MODE_SHIFT) /* 3: 1 Mbit/s BLE */
#define RADIO_MODE_BLE2MBIT               (0x04 << RADIO_MODE_SHIFT) /* 4: 2 Mbit/s BLE */
#ifdef HAVE_RADIO_BLELR
#  define RADIO_MODE_BLELR125KBIT         (0x05 << RADIO_MODE_SHIFT) /* 5: Long range 125 kbit/s TX, 125 kbit/s and 500 kbit/s RX */
#  define RADIO_MODE_BLELR500KBIT         (0x06 << RADIO_MODE_SHIFT) /* 6: Long range 500 kbit/s TX, 125 kbit/s and 500 kbit/s RX */
#endif
#ifdef HAVE_RADIO_IEEE802154
#  define RADIO_MODE_IEEE802154           (0x0f << RADIO_MODE_SHIFT) /* 15: IEEE 802.15.4-2006 250 kbit/s */
#endif

/* PCNF0 Register */

#define RADIO_PCNF0_LFLEN_SHIFT           (0)       /* Bits 0-3: Length on air of LENGTH field in number of bits */
#define RADIO_PCNF0_LFLEN_MASK            (0xf << RADIO_PCNF0_LFLEN_SHIFT)
#define RADIO_PCNF0_LFLEN_MAX             (0xf)
#define RADIO_PCNF0_S0LEN_SHIFT           (8)      /* Bit 8: Length on air of S0 field in number of bytes */
#define RADIO_PCNF0_S0LEN_MASK            (1 << RADIO_PCNF0_S0LEN_SHIFT)
#define RADIO_PCNF0_S0LEN_MAX             (1)
#define RADIO_PCNF0_S1LEN_SHIFT           (16)      /* Bits 16-19: Length on air of S1 field in number of bits */
#define RADIO_PCNF0_S1LEN_MASK            (0xf << RADIO_PCNF0_S1LEN_SHIFT)
#define RADIO_PCNF0_S1LEN_MAX             (0xf)
#define RADIO_PCNF0_S1INCL                (1 << 20) /* Bit 20: Include or exclude S1 field in RAM */
#ifdef HAVE_RADIO_BLELR
#  define RADIO_PCNF0_CILEN_SHIFT         (22)      /* Bits 22-23: Length of code indicator - long range */
#  define RADIO_PCNF0_CILEN_MASK          (0x3 << RADIO_PCNF0_CILEN_SHIFT)
#  define RADIO_PCNF0_CILEN_MAX           (0x3)
#endif

#define RADIO_PCNF0_PLEN_SHIFT            (24)      /* Bits 24-25: Length of preamble on air */
#define RADIO_PCNF0_PLEN_MASK             (0x3 << RADIO_PCNF0_PLEN_SHIFT)
#  define RADIO_PCNF0_PLEN_8BIT           (0 << RADIO_PCNF0_PLEN_SHIFT)
#  define RADIO_PCNF0_PLEN_16BIT          (1 << RADIO_PCNF0_PLEN_SHIFT)
#ifdef HAVE_RADIO_IEEE802154
#  define RADIO_PCNF0_PLEN_32BITZ         (2 << RADIO_PCNF0_PLEN_SHIFT)
#endif
#ifdef HAVE_RADIO_BLELR
#  define RADIO_PCNF0_PLEN_LONGRANGE      (3 << RADIO_PCNF0_PLEN_SHIFT)
#endif
#ifdef HAVE_RADIO_BLELR
#  define RADIO_PCNF0_CRCINC_SHIFT        (26)      /* Bit 26: Indicates if LENGTH field contains CRC */
#  define RADIO_PCNF0_CRCINC              (1 << RADIO_PCNF0_CRCINC_SHIFT)
#  define RADIO_PCNF0_TERMLEN_SHIFT       (29)      /* Bits 29-30: Length of TERM field in Long Range operation */
#  define RADIO_PCNF0_TERMLEN_MASK        (0x3 << RADIO_PCNF0_TERMLEN_SHIFT)
#  define RADIO_PCNF0_TERMLEN_MAX         (0x3)
#endif

/* PCNF1 Register */

#define RADIO_PCNF1_MAXLEN_SHIFT          (0)       /* Bits 0-7: Maximum length of packet payload */
#define RADIO_PCNF1_MAXLEN_MASK           (0xff << RADIO_PCNF1_MAXLEN_SHIFT)
#define RADIO_PCNF1_MAXLEN_MAX            (0xff)
#define RADIO_PCNF1_STATLEN_SHIFT         (8)       /* Bits 8-15: Static length in number of bytes */
#define RADIO_PCNF1_STATLEN_MASK          (0xff << RADIO_PCNF1_STATLEN_SHIFT)
#define RADIO_PCNF1_STATLEN_MAX           (0xff)
#define RADIO_PCNF1_BALEN_SHIFT           (16)      /* Bits 16-18: Base address length in number of bytes */
#define RADIO_PCNF1_BALEN_MASK            (0x7 << RADIO_PCNF1_BALEN_SHIFT)
#define RADIO_PCNF1_BALEN_MIN             (0x2)
#define RADIO_PCNF1_BALEN_MAX             (0x4)
#define RADIO_PCNF1_ENDIAN_SHIFT          (24)      /* Bit 24: On air endianness of packet */
#define RADIO_PCNF1_ENDIAN                (1 << RADIO_PCNF1_ENDIAN_SHIFT)
#  define RADIO_PCNF1_ENDIAN_LITTLE       (0 << 24) /* Least significant bit on air first */
#  define RADIO_PCNF1_ENDIAN_BITG         (1 << 24) /* Most significant bit on air first */
#define RADIO_PCNF1_WHITEEN_SHIFT         (25)      /* Bit 25: Enable or disable packet whitening */
#define RADIO_PCNF1_WHITEEN               (1 << RADIO_PCNF1_WHITEEN_SHIFT)

/* TXADDRESS Register */

#define RADIO_TXADDRESS_SHIFT             (0)       /* Bits 0-2: Transmit address select */
#define RADIO_TXADDRESS_MASK              (0x7 << RADIO_TXADDRESS_SHIFT)

/* RXADDRESSES Register */

#define RADIO_RXADDRESSES_ADDR(i)         (1 << (i)) /* Bits 0-7: Enable or disable reception on logical address i */

/* CRCCNF Register */

#define RADIO_CRCCNF_LEN_SHIFT            (0)       /* Bit 0-1: CRC length in number of bytes */
#define RADIO_CRCCNF_LEN_MASK             (0x3 << RADIO_CRCCNF_LEN_SHIFT)
#  define RADIO_CRCCNF_LEN_DIS            (0 << RADIO_CRCCNF_LEN_SHIFT)
#  define RADIO_CRCCNF_LEN_1              (1 << RADIO_CRCCNF_LEN_SHIFT)
#  define RADIO_CRCCNF_LEN_2              (2 << RADIO_CRCCNF_LEN_SHIFT)
#  define RADIO_CRCCNF_LEN_3              (3 << RADIO_CRCCNF_LEN_SHIFT)
#define RADIO_CRCCNF_SKIPADDR_SHIFT       (8)       /* Bit 8-10: Include or exclude packet address field out of CRC calculation */
#ifdef HAVE_RADIO_IEEE802154
#  define RADIO_CRCCNF_SKIPADDR_MASK      (0x7 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#  define RADIO_CRCCNF_SKIPADDR_INCL      (0 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#  define RADIO_CRCCNF_SKIPADDR_SKIP      (1 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#  define RADIO_CRCCNF_SKIPADDR_IEEE      (2 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#else
#  define RADIO_CRCCNF_SKIPADDR_MASK      (0x1 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#  define RADIO_CRCCNF_SKIPADDR_INCL      (0 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#  define RADIO_CRCCNF_SKIPADDR_SKIP      (1 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#endif

/* CRCPOLY Register */

#define RADIO_CRCPOLY_SHIFT               (0)      /* Bits 0-23: CRC polynomial */
#define RADIO_CRCPOLY_MASK                (0x00ffffff << RADIO_CRCPOLY_SHIFT)

/* CRCINIT Register */

#define RADIO_CRCINIT_SHIFT               (0)       /* Bits 0-23: CRC initial value */
#define RADIO_CRCINIT_MASK                (0x00ffffff << RADIO_CRCINIT_SHIFT)

/* TIFS Register */

#define RADIO_TIFS_SHIFT                  (0)      /* Bits 0-9: Interframe spacing in us */
#define RADIO_TIFS_MASK                   (0x3ff << RADIO_TIFS_SHIFT)
#define RADIO_TIFS_MAX                    (0x3ff)

/* RSSISAMPLE Register */

#define RADIO_RSSISAMPLE_SHIFT            (0)     /* Bits 0-6: RSSI sample */
#define RADIO_RSSISAMPLE_MASK             (0x7f << RADIO_RSSISAMPLE_SHIFT)

/* STATE Register */

#define RADIO_STATE_SHIFT                 (0)     /* Bits 0-3: Current radio state */
#define RADIO_STATE_MASK                  (0xf << RADIO_STATE_SHIFT)
#  define RADIO_STATE_DISABLED            (0 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_RXRU                (1 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_RXIDLE              (2 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_RX                  (3 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_RXDISABLE           (4 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_SETTLE              (5 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_PLL                 (6 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_TXRU                (9 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_TXIDLE              (10 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_TX                  (11 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_TXDISABLE           (12 << RADIO_STATE_SHIFT)

/* DATAWHITE Register */

#define RADIO_DATAWHITE_IV_SHIFT          (0)     /* Bits 0-8: Data whitening initial value */
#define RADIO_DATAWHITE_IV_MASK           (0x1ff << RADIO_DATAWHITE_IV_SHIFT)
#define RADIO_DATAWHITE_IV_MAX            (0x1ff)
#define RADIO_DATAWHITE_POLY_SHIFT        (16)    /* Bits 16-25: Data whitening polynomial */
#define RADIO_DATAWHITE_POLY_MASK         (0x3ff << RADIO_DATAWHITE_POLY_SHIFT)
#define RADIO_DATAWHITE_POLY_BLE          (0x089 << RADIO_DATAWHITE_POLY_SHIFT) /* Reset value */

/* DAP Register */

#define RADIO_DAP_SHIFT                   (0)     /* Bits 0-15: Device address prefix n */
#define RADIO_DAP_MASK                    (0xffff << RADIO_DAP_SHIFT)

/* DANCF Register */

#define RADIO_DACNF_ENA(i)                (1 << (i))     /* Bits 0-7: Enable device address matching using */
#define RADIO_DACNF_TXADD(i)              (1 << (i + 8)) /* Bits 8-15: TXADD for device address i */

/* TIMING Register */

#define RADIO_TIMING_RU                   (1 << 0) /* Bit 0: Radio ramp-up time */

#ifdef HAVE_RADIO_IEEE802154
/* SFD Register */

#  define RADIO_SFD_SHIFT                 (0)     /* Bits 0-7: IEEE 802.15.4 start of frame delimiter */
#  define RADIO_SFD_MASK                  (0xff << RADIO_SFD_SHIFT)

/* EDCTRL Register */

#  define RADIO_EDCNT_SHIFT               (0)     /* Bits 0-20: IEEE 802.15.4 energy detect loop count */
#  define RADIO_EDCNT_MASK                (0x1fffff << RADIO_EDCNT_SHIFT)
#  define RADIO_EDPERIOD_SHIFT            (24)    /* Bits 24-29: IEEE 802.15.4 energy detect period */
#  define RADIO_EDPERIOD_MASK             (0x3f << RADIO_EDPERIOD_SHIFT)

/* EDSAMPLE Register */

#  define RADIO_EDSAMPLE_SHIFT            (0)     /* Bits 0-7: IEEE 802.15.4 energy detect level */
#  define RADIO_EDSAMPLE_MASK             (0xff << RADIO_EDSAMPLE_SHIFT)
#endif  /* HAVE_RADIO_IEEE802154 */

/* CCACTRL Register */

#define RADIO_CCACTRL_CCAMODE_SHIFT       (0)     /* Bits 0-3: CCA mode of operation */
#define RADIO_CCACTRL_CCAMODE_MASK        (0x7 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_ED        (0 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_C         (1 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_CANDED    (2 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_CORED     (3 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_EDTST1    (4 << RADIO_CCACTRL_CCAMODE_SHIFT)
#define RADIO_CCACTRL_CCAEDTHRES_SHIFT    (8)
#define RADIO_CCACTRL_CCAEDTHRES_MASK     (0xff << RADIO_CCACTRL_CCAEDTHRES_SHIFT)
#define RADIO_CCACTRL_CCACORRTHRES_SHIFT  (16)
#define RADIO_CCACTRL_CCACORRTHRES_MASK   (0xff << RADIO_CCACTRL_CCACORRTHRES_SHIFT)
#define RADIO_CCACTRL_CCACORRCNT_SHIFT    (24)
#define RADIO_CCACTRL_CCACORRCNT_MASK     (0xff << RADIO_CCACTRL_CCACORRCNT_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_RADIO_H */
