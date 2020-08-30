/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_radio.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_RADIO_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_RADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_ARCH_CHIP_NRF52832)
#  define HAVE_RADIO_NRF250KBIT
#  undef  HAVE_RADIO_BLELR
#  undef  HAVE_RADIO_IEEE802154
#elif defined(CONFIG_ARCH_CHIP_NRF52833)
#  undef  HAVE_RADIO_NRF250KBIT
#  define HAVE_RADIO_BLELR
#  define HAVE_RADIO_IEEE802154
#elif defined(CONFIG_ARCH_CHIP_NRF52840)
#  undef  HAVE_RADIO_NRF250KBIT
#  define HAVE_RADIO_BLELR
#  define HAVE_RADIO_IEEE802154
#else
#  error Unknown NRF52 chip !
#endif

/* Register offsets *********************************************************/

#define NRF52_RADIO_TASKS_TXEN_OFFSET        0x0000                 /* Enable RADIO in TX mode */
#define NRF52_RADIO_TASKS_RXEN_OFFSET        0x0004                 /* Enable RADIO in RX mode */
#define NRF52_RADIO_TASKS_START_OFFSET       0x0008                 /* Start RADIO */
#define NRF52_RADIO_TASKS_STOP_OFFSET        0x000C                 /* Stop RADIO */
#define NRF52_RADIO_TASKS_DISABLE_OFFSET     0x0010                 /* Disable RADIO */
#define NRF52_RADIO_TASKS_RSSISTART_OFFSET   0x0014                 /* Start the RSSI and take one single sample of the receive signal strength */
#define NRF52_RADIO_TASKS_RSSISTOP_OFFSET    0x0018                 /* Stop the RSSI measurement */
#define NRF52_RADIO_TASKS_BCSTART_OFFSET     0x001c                 /* Start the bit counter */
#define NRF52_RADIO_TASKS_BCSTOP_OFFSET      0x0020                 /* Stop the bit counter */
#define NRF52_RADIO_TASKS_EDSTART_OFFSET     0x0024                 /* Start the energy detect measurement used in IEEE 802.15.4 mode */
#define NRF52_RADIO_TASKS_EDSTOP_OFFSET      0x0028                 /* Stop the energy detect measurement  */
#define NRF52_RADIO_TASKS_CCASTART_OFFSET    0x002c                 /* Start the clear channel assessment used in IEEE 802.15.4 mode */
#define NRF52_RADIO_TASKS_CCASTOP_OFFSET     0x0030                 /* Stop the clear channel assessment */
#define NRF52_RADIO_EVENTS_READY_OFFSET      0x0100                 /* RADIO has ramped up and is ready to be started */
#define NRF52_RADIO_EVENTS_ADDRESS_OFFSET    0x0104                 /* Address sent or received */
#define NRF52_RADIO_EVENTS_PAYLOAD_OFFSET    0x0108                 /* Packet payload sent or received */
#define NRF52_RADIO_EVENTS_END_OFFSET        0x010c                 /* Packet sent or received */
#define NRF52_RADIO_EVENTS_DISABLED_OFFSET   0x0110                 /* RADIO has been disabled */
#define NRF52_RADIO_EVENTS_DEVMATCH_OFFSET   0x0114                 /* A device address match occurred on the last received packet */
#define NRF52_RADIO_EVENTS_DEVMISS_OFFSET    0x0118                 /* No device address match occurred on the last received packet */
#define NRF52_RADIO_EVENTS_RSSIEND_OFFSET    0x011c                 /* Sampling of receive signal strength complete */
#define NRF52_RADIO_EVENTS_BCMATCH_OFFSET    0x0128                 /* Bit counter reached bit count value */
#define NRF52_RADIO_EVENTS_CRCOK_OFFSET      0x0130                 /* Packet received with CRC ok */
#define NRF52_RADIO_EVENTS_CRCERROR_OFFSET   0x0134                 /* Packet received with CRC error */
#define NRF52_RADIO_EVENTS_FRAMESTART_OFFSET 0x0138                 /* IEEE 802.15.4 length field received */
#define NRF52_RADIO_EVENTS_EDEND_OFFSET      0x013c                 /* Sampling of energy detection complete */
#define NRF52_RADIO_EVENTS_EDSTOPPED_OFFSET  0x0140                 /* The sampling of energy detection has stopped */
#define NRF52_RADIO_EVENTS_CCAIDLE_OFFSET    0x0144                 /* Wireless medium in idle - clear to send */
#define NRF52_RADIO_EVENTS_CCABUSY_OFFSET    0x0148                 /* Wireless medium busy - do not send */
#define NRF52_RADIO_EVENTS_CCASTOPPED_OFFSET 0x014c                 /* The CCA has stopped */
#define NRF52_RADIO_EVENTS_RATEBOOST_OFFSET  0x0150                 /* Ble_LR CI field received */
#define NRF52_RADIO_EVENTS_TXREADY_OFFSET    0x0154                 /* RADIO has ramped up and is ready to be started TX path */
#define NRF52_RADIO_EVENTS_RXREADY_OFFSET    0x0158                 /* RADIO has ramped up and is ready to be started RX path */
#define NRF52_RADIO_EVENTS_MHRMATCH_OFFSET   0x015c                 /* MAC header match found */
#define NRF52_RADIO_EVENTS_PHYEND_OFFSET     0x016c                 /* Last bit is sent on air */
#define NRF52_RADIO_SHORTS_OFFSET            0x0200                 /* Shortcuts between local events and tasks */
#define NRF52_RADIO_INTENSET_OFFSET          0x0304                 /* Enable interrupt */
#define NRF52_RADIO_INTENCLR_OFFSET          0x0308                 /* Disable interrupt */
#define NRF52_RADIO_CRCSTATUS_OFFSET         0x0400                 /* CRC status */
#define NRF52_RADIO_RXMATCH_OFFSET           0x0408                 /* Received address */
#define NRF52_RADIO_RXCRC_OFFSET             0x040c                 /* CRC field of previously received packet */
#define NRF52_RADIO_DAI_OFFSET               0x0410                 /* Device address match index */
#define NRF52_RADIO_PDUSTAT_OFFSET           0x0414                 /* Payload status */
#define NRF52_RADIO_PACKETPTR_OFFSET         0x0504                 /* Packet pointer */
#define NRF52_RADIO_FREQUENCY_OFFSET         0x0508                 /* Frequency */
#define NRF52_RADIO_TXPOWER_OFFSET           0x050c                 /* Output power */
#define NRF52_RADIO_MODE_OFFSET              0x0510                 /* Data rate and modulation */
#define NRF52_RADIO_PCNF0_OFFSET             0x0514                 /* Packet configuration register 0 */
#define NRF52_RADIO_PCNF1_OFFSET             0x0518                 /* Packet configuration register 1 */
#define NRF52_RADIO_BASE0_OFFSET             0x051c                 /* Base address 0 */
#define NRF52_RADIO_BASE1_OFFSET             0x0520                 /* Base address 1 */
#define NRF52_RADIO_PREFIX0_OFFSET           0x0524                 /* Prefixes bytes for logical addresses 0-3 */
#define NRF52_RADIO_PREFIX1_OFFSET           0x0528                 /* Prefixes bytes for logical addresses 4-7 */
#define NRF52_RADIO_TXADDRESS_OFFSET         0x052c                 /* Transmit address select */
#define NRF52_RADIO_RXADDRESSES_OFFSET       0x0530                 /* Receive address select */
#define NRF52_RADIO_CRCCNF_OFFSET            0x0534                 /* CRC configuration */
#define NRF52_RADIO_CRCPOLY_OFFSET           0x0538                 /* CRC polynomial */
#define NRF52_RADIO_CRCINIT_OFFSET           0x053c                 /* CRC initial value */
#define NRF52_RADIO_TIFS_OFFSET              0x0544                 /* Interframe spacing in μs */
#define NRF52_RADIO_RSSISAMPLE_OFFSET        0x0548                 /* RSSI sample */
#define NRF52_RADIO_STATE_OFFSET             0x0550                 /* Current radio state */
#define NRF52_RADIO_DATAWHITEIV_OFFSET       0x0554                 /* Data whitening initial value */
#define NRF52_RADIO_BCC_OFFSET               0x0560                 /* Bit counter compare */
#define NRF52_RADIO_DAB_OFFSET(p)            (0x0600 + ((p) * 0x4)) /* Device address base segment */
#define NRF52_RADIO_DAP_OFFSET(p)            (0x0620 + ((p) * 0x4)) /* Device address prefix */
#define NRF52_RADIO_DACNF_OFFSET             0x0640                 /* Device address match configuration */
#define NRF52_RADIO_MHRMATCHCONF_OFFSET      0x0644                 /* Search pattern configuration */
#define NRF52_RADIO_MHRMATCHMAS_OFFSET       0x0648                 /* Pattern mask */
#define NRF52_RADIO_MODECNF0_OFFSET          0x0650                 /* Radio mode configuration register 0 */
#define NRF52_RADIO_SFD_OFFSET               0x0660                 /* IEEE 802.15.4 start of frame delimiter */
#define NRF52_RADIO_EDCNT_OFFSET             0x0664                 /* IEEE 802.15.4 energy detect loop count */
#define NRF52_RADIO_EDSAMPLE_OFFSET          0x0668                 /* IEEE 802.15.4 energy detect level */
#define NRF52_RADIO_CCACTRL_OFFSET           0x066c                 /* IEEE 802.15.4 clear channel assessment control */
#define NRF52_RADIO_POWER_OFFSET             0x0ffc                 /* Peripheral power control */

/* Register Addresses *******************************************************/

#define NRF52_RADIO_TASKS_TXEN        (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_TXEN_OFFSET)
#define NRF52_RADIO_TASKS_RXEN        (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_RXEN_OFFSET)
#define NRF52_RADIO_TASKS_START       (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_START_OFFSET)
#define NRF52_RADIO_TASKS_STOP        (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_STOP_OFFSET)
#define NRF52_RADIO_TASKS_DISABLE     (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_DISABLE_OFFSET)
#define NRF52_RADIO_TASKS_RSSISTART   (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_RSSISTART_OFFSET)
#define NRF52_RADIO_TASKS_RSSISTOP    (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_RSSISTOP_OFFSET)
#define NRF52_RADIO_TASKS_BCSTART     (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_BCSTART_OFFSET)
#define NRF52_RADIO_TASKS_BCSTOP      (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_BCSTOP_OFFSET)
#define NRF52_RADIO_TASKS_EDSTART     (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_EDSTART_OFFSET)
#define NRF52_RADIO_TASKS_EDSTOP      (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_EDSTOP_OFFSET)
#define NRF52_RADIO_TASKS_CCASTART    (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_CCASTART_OFFSET)
#define NRF52_RADIO_TASKS_CCASTOP     (NRF52_RADIO_BASE + NRF52_RADIO_TASKS_CCASTOP_OFFSET)
#define NRF52_RADIO_EVENTS_READY      (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_READY_OFFSET)
#define NRF52_RADIO_EVENTS_ADDRESS    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_ADDRESS_OFFSET)
#define NRF52_RADIO_EVENTS_PAYLOAD    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_PAYLOAD_OFFSET)
#define NRF52_RADIO_EVENTS_END        (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_END_OFFSET)
#define NRF52_RADIO_EVENTS_DISABLED   (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_DISABLED_OFFSET)
#define NRF52_RADIO_EVENTS_DEVMATCH   (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_DEVMATCH_OFFSET)
#define NRF52_RADIO_EVENTS_DEVMISS    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_DEVMISS_OFFSET)
#define NRF52_RADIO_EVENTS_RSSIEND    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_RSSIEND_OFFSET)
#define NRF52_RADIO_EVENTS_BCMATCH    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_BCMATCH_OFFSET)
#define NRF52_RADIO_EVENTS_CRCOK      (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_CRCOK_OFFSET)
#define NRF52_RADIO_EVENTS_CRCERROR   (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_CRCERROR_OFFSET)
#define NRF52_RADIO_EVENTS_FRAMESTART (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_FRAMESTART_OFFSET)
#define NRF52_RADIO_EVENTS_EDEND      (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_EDEND_OFFSET)
#define NRF52_RADIO_EVENTS_EDSTOPPED  (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_EDSTOPPED_OFFSET)
#define NRF52_RADIO_EVENTS_CCAIDLE    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_CCAIDLE_OFFSET)
#define NRF52_RADIO_EVENTS_CCABUSY    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_CCABUSY_OFFSET)
#define NRF52_RADIO_EVENTS_CCASTOPPED (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_CCASTOPPED_OFFSET)
#define NRF52_RADIO_EVENTS_RATEBOOST  (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_RATEBOOST_OFFSET)
#define NRF52_RADIO_EVENTS_TXREADY    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_TXREADY_OFFSET)
#define NRF52_RADIO_EVENTS_RXREADY    (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_RXREADY_OFFSET)
#define NRF52_RADIO_EVENTS_MHRMATCH   (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_MHRMATCH_OFFSET)
#define NRF52_RADIO_EVENTS_PHYEND     (NRF52_RADIO_BASE + NRF52_RADIO_EVENTS_PHYEND_OFFSET)
#define NRF52_RADIO_SHORTS            (NRF52_RADIO_BASE + NRF52_RADIO_SHORTS_OFFSET)
#define NRF52_RADIO_INTENSET          (NRF52_RADIO_BASE + NRF52_RADIO_INTENSET_OFFSET)
#define NRF52_RADIO_INTENCLR          (NRF52_RADIO_BASE + NRF52_RADIO_INTENCLR_OFFSET)
#define NRF52_RADIO_CRCSTATUS         (NRF52_RADIO_BASE + NRF52_RADIO_CRCSTATUS_OFFSET)
#define NRF52_RADIO_RXMATCH           (NRF52_RADIO_BASE + NRF52_RADIO_RXMATCH_OFFSET)
#define NRF52_RADIO_RXCRC             (NRF52_RADIO_BASE + NRF52_RADIO_RXCRC_OFFSET)
#define NRF52_RADIO_DAI               (NRF52_RADIO_BASE + NRF52_RADIO_DAI_OFFSET)
#define NRF52_RADIO_PDUSTAT           (NRF52_RADIO_BASE + NRF52_RADIO_PDUSTAT_OFFSET)
#define NRF52_RADIO_PACKETPTR         (NRF52_RADIO_BASE + NRF52_RADIO_PACKETPTR_OFFSET)
#define NRF52_RADIO_FREQUENCY         (NRF52_RADIO_BASE + NRF52_RADIO_FREQUENCY_OFFSET)
#define NRF52_RADIO_TXPOWER           (NRF52_RADIO_BASE + NRF52_RADIO_TXPOWER_OFFSET)
#define NRF52_RADIO_MODE              (NRF52_RADIO_BASE + NRF52_RADIO_MODE_OFFSET)
#define NRF52_RADIO_PCNF0             (NRF52_RADIO_BASE + NRF52_RADIO_PCNF0_OFFSET)
#define NRF52_RADIO_PCNF1             (NRF52_RADIO_BASE + NRF52_RADIO_PCNF1_OFFSET)
#define NRF52_RADIO_BASE0             (NRF52_RADIO_BASE + NRF52_RADIO_BASE0_OFFSET)
#define NRF52_RADIO_BASE1             (NRF52_RADIO_BASE + NRF52_RADIO_BASE1_OFFSET)
#define NRF52_RADIO_PREFIX0           (NRF52_RADIO_BASE + NRF52_RADIO_PREFIX0_OFFSET)
#define NRF52_RADIO_PREFIX1           (NRF52_RADIO_BASE + NRF52_RADIO_PREFIX1_OFFSET)
#define NRF52_RADIO_TXADDRESS         (NRF52_RADIO_BASE + NRF52_RADIO_TXADDRESS_OFFSET)
#define NRF52_RADIO_RXADDRESSES       (NRF52_RADIO_BASE + NRF52_RADIO_RXADDRESSES_OFFSET)
#define NRF52_RADIO_CRCCNF            (NRF52_RADIO_BASE + NRF52_RADIO_CRCCNF_OFFSET)
#define NRF52_RADIO_CRCPOLY           (NRF52_RADIO_BASE + NRF52_RADIO_CRCPOLY_OFFSET)
#define NRF52_RADIO_CRCINIT           (NRF52_RADIO_BASE + NRF52_RADIO_CRCINIT_OFFSET)
#define NRF52_RADIO_TIFS              (NRF52_RADIO_BASE + NRF52_RADIO_TIFS_OFFSET)
#define NRF52_RADIO_RSSISAMPLE        (NRF52_RADIO_BASE + NRF52_RADIO_RSSISAMPLE_OFFSET)
#define NRF52_RADIO_STATE             (NRF52_RADIO_BASE + NRF52_RADIO_STATE_OFFSET)
#define NRF52_RADIO_DATAWHITEIV       (NRF52_RADIO_BASE + NRF52_RADIO_DATAWHITEIV_OFFSET)
#define NRF52_RADIO_BCC               (NRF52_RADIO_BASE + NRF52_RADIO_BCC_OFFSET)
#define NRF52_RADIO_DAB(p)            (NRF52_RADIO_BASE + NRF52_RADIO_DAB_OFFSET(p))
#define NRF52_RADIO_DAP(p)            (NRF52_RADIO_BASE + NRF52_RADIO_PAP_OFFSET(p))
#define NRF52_RADIO_DACNF             (NRF52_RADIO_BASE + NRF52_RADIO_DACNF_OFFSET)
#define NRF52_RADIO_MHRMATCHCONF      (NRF52_RADIO_BASE + NRF52_RADIO_MHRMATCHCONF_OFFSET)
#define NRF52_RADIO_MHRMATCHMAS       (NRF52_RADIO_BASE + NRF52_RADIO_MHRMATCHMAS_OFFSET)
#define NRF52_RADIO_MODECNF0          (NRF52_RADIO_BASE + NRF52_RADIO_MODECNF0_OFFSET)
#define NRF52_RADIO_SFD               (NRF52_RADIO_BASE + NRF52_RADIO_SFD_OFFSET)
#define NRF52_RADIO_EDCNT             (NRF52_RADIO_BASE + NRF52_RADIO_EDCNT_OFFSET)
#define NRF52_RADIO_EDSAMPLE          (NRF52_RADIO_BASE + NRF52_RADIO_EDSAMPLE_OFFSET)
#define NRF52_RADIO_CCACTRL           (NRF52_RADIO_BASE + NRF52_RADIO_CCACTRL_OFFSET)
#define NRF52_RADIO_POWER             (NRF52_RADIO_BASE + NRF52_RADIO_POWER_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* TASKS_TXEN Register */

#define RADIO_TASKS_TXEN                (1 << 0)    /* Bit 0: Enable RADIO in TX mode */

/* TASKS_RXEN Register */

#define RADIO_TASKS_RXEN                (1 << 0)    /* Bit 0: Enable RADIO in RX mode */

/* TASKS_START Register */

#define RADIO_TASKS_START               (1 << 0)    /* Bit 0: Start RADIO */

/* TASKS_STOP Register */

#define RADIO_TASKS_STOP                (1 << 0)    /* Bit 0: Stop RADIO */

/* TASKS_DISABLE Register */

#define RADIO_TASKS_DISABLE             (1 << 0)    /* Bit 0: Disable RADIO */

/* TASKS_RSSISTART Register */

#define RADIO_TASKS_RSSISTART           (1 << 0)    /* Bit 0: Start the RSSI */

/* TASKS_RSSISTOP Register */

#define RADIO_TASKS_RSSISTOP            (1 << 0)    /* Bit 0: Stop the RSSI */

/* TASKS_BCSTART Register */

#define RADIO_TASKS_BCSTART             (1 << 0)    /* Bit 0: Start the bit counter */

/* TASKS_BCSTOP Register */

#define RADIO_TASKS_BCSTOP              (1 << 0)    /* Bit 0: Stop the bit counter */

/* TASKS_EDSTART Register */

#define RADIO_TASKS_EDSTART             (1 << 0)    /* Bit 0: Start the energy detect measurement (IEEE 802.15.4) */

/* TASKS_EDSTOP Register */

#define RADIO_TASKS_EDSTOP              (1 << 0)    /* Bit 0: Stop the energy detect measurement (IEEE 802.15.4) */

/* TASKS_CCASTART Register */

#define RADIO_TASKS_CCASTART            (1 << 0)    /* Bit 0: Start the channel assessment (IEEE 802.15.4) */

/* TASKS_CCASTOP Register */

#define RADIO_TASKS_CCASTOP             (1 << 0)    /* Bit 0: Stop the channel assessment (IEEE 802.15.4) */

/* EVENTS_READY Register */

#define RADIO_EVENTS_READY              (1 << 0)    /* Bit 0: RADIO has ramped up and is ready to be started */

/* EVENTS_ADDRESS Register */

#define RADIO_EVENTS_ADDRESS            (1 << 0)    /* Bit 0: Address sent or received */

/* EVENTS_PAYLOAD Register */

#define RADIO_EVENTS_PAYLOAD            (1 << 0)    /* Bit 0: Packet payload sent or received */

/* EVENTS_END Register */

#define RADIO_EVENTS_END                (1 << 0)    /* Bit 0: Packet sent or received */

/* EVENTS_DISABLED Register */

#define RADIO_EVENTS_DISABLED           (1 << 0)    /* Bit 0: RADIO has been disabled */

/* EVENTS_DEVMATCH Register */

#define RADIO_EVENTS_DEVMATCH           (1 << 0)    /* Bit 0: A device address match */

/* EVENTS_DEVMISS Register */

#define RADIO_EVENTS_DEVMISS            (1 << 0)    /* Bit 0: No device address match */

/* EVENTS_RSSIEND Register */

#define RADIO_EVENTS_RSSIEND            (1 << 0)    /* Bit 0: Sampling of receive signal strength complete */

/* EVENTS_BCMATCH Register */

#define RADIO_EVENTS_BCMATCH            (1 << 0)    /* Bit 0: Bit counter reached bit count value */

/* EVENTS_CRCOK Register */

#define RADIO_EVENTS_CRCOK              (1 << 0)    /* Bit 0: Packet received with CRC ok */

/* EVENTS_CRCERROR Register */

#define RADIO_EVENTS_CRCERROR           (1 << 0)    /* Bit 0: Packet received with CRC error */

/* EVENTS_FRAMESTART Register */

#define RADIO_EVENTS_FRAMESTART         (1 << 0)    /* Bit 0: IEEE 802.15.4 length field received*/

/* EVENTS_EDEND Register */

#define RADIO_EVENTS_EDEND              (1 << 0)    /* Bit 0: ampling of energy detection complete */

/* EVENTS_EDSTOPPED Register */

#define RADIO_EVENTS_EDSTOPPED          (1 << 0)    /* Bit 0: The sampling of energy detection has stopped */

/* EVENTS_CCAIDLE Register */

#define RADIO_EVENTS_CCAIDLE            (1 << 0)    /* Bit 0: Wireless medium in idle */

/* EVENTS_CCABUSY Register */

#define RADIO_EVENTS_CCABUSY            (1 << 0)    /* Bit 0: Wireless medium busy */

/* EVENTS_CCASTOPPED Register */

#define RADIO_EVENTS_CCASTOPPED         (1 << 0)    /* Bit 0: The CCA has stopped */

/* EVENTS_RATEBOOST Register */

#define RADIO_EVENTS_RATEBOOST          (1 << 0)    /* Bit 0: Ble_LR CI field received */

/* EVENTS_TXREADY Register */

#define RADIO_EVENTS_TXREADY            (1 << 0)    /* Bit 0: RADIO has ramped up and is ready to be started TX path */

/* EVENTS_RXREADY Register */

#define RADIO_EVENTS_RXREADY            (1 << 0)    /* Bit 0: RADIO has ramped up and is ready to be started RX path */

/* EVENTS_MHRMATCH Register */

#define RADIO_EVENTS_MHRMATCH           (1 << 0)    /* Bit 0: MAC header match found */

/* EVENTS_PHYEND Register */

#define RADIO_EVENTS_PHYEND             (1 << 0)    /* Bit 0: Last bit is sent on air */

/* SHORTS Register */

#define RADIO_SHORTS_READY_START        (1 << 0)    /* Bit 0: Shortcut between event READY and task START */
#define RADIO_SHORTS_END_DISABLE        (1 << 1)    /* Bit 1: Shortcut between event END and task DISABLE */
#define RADIO_SHORTS_DISABLED_TXEN      (1 << 2)    /* Bit 2: Shortcut between event DISABLED and task TXEN */
#define RADIO_SHORTS_DISABLED_RXEN      (1 << 3)    /* Bit 3: Shortcut between event DISABLED and task RXEN */
#define RADIO_SHORTS_ADDRESS_RSSISTART  (1 << 4)    /* Bit 4: Shortcut between event ADDRESS and task RSSISTART */
#define RADIO_SHORTS_END_START          (1 << 5)    /* Bit 5: Shortcut between event END and task START */
#define RADIO_SHORTS_ADDRESS_BCSTART    (1 << 6)    /* Bit 6: Shortcut between event ADDRESS and task BCSTART */
#define RADIO_SHORTS_DISABLED_RSSISTOP  (1 << 8)    /* Bit 8: Shortcut between event DISABLED and task RSSISTOP */
#define RADIO_SHORTS_RXREADY_CCASTART   (1 << 11)   /* Bit 11: Shortcut between event RXREADY and task CCASTART */
#define RADIO_SHORTS_CCAIDLE_TXEN       (1 << 12)   /* Bit 12: Shortcut between event CCAIDLE and task TXEN */
#define RADIO_SHORTS_CCABUSY_DISABLE    (1 << 13)   /* Bit 13: Shortcut between event CCABUSY and task DISABLE */
#define RADIO_SHORTS_FRAMESTART_BCSTART (1 << 14)   /* Bit 14: Shortcut between event FRAMESTART and task BCSTART */
#define RADIO_SHORTS_READY_EDSTART      (1 << 15)   /* Bit 15: Shortcut between event READY and task EDSTART */
#define RADIO_SHORTS_EDEND_DISABLE      (1 << 16)   /* Bit 16: Shortcut between event EDEND and task DISABLE */
#define RADIO_SHORTS_CCAIDLE_STOP       (1 << 17)   /* Bit 17: Shortcut between event CCAIDLE and task STOP */
#define RADIO_SHORTS_TXREADY_START      (1 << 18)   /* Bit 18: Shortcut between event TXREADY and task START */
#define RADIO_SHORTS_RXREADY_START      (1 << 19)   /* Bit 19: Shortcut between event RXREADY and task START */
#define RADIO_SHORTS_PHYEND_DISABLE     (1 << 20)   /* Bit 20: Shortcut between event PHYEND and task DISABLE */
#define RADIO_SHORTS_PHYEND_START       (1 << 21)   /* Bit 21: Shortcut between event PHYEND and task START */

/* INTENSET/INTENCLR Register */

#define RADIO_INT_READY                 (1 << 0)    /* Bit 0: Interrupt for event READY */
#define RADIO_INT_ADDRESS               (1 << 1)    /* Bit 1: Interrupt for event ADDRESS */
#define RADIO_INT_PAYLOAD               (1 << 2)    /* Bit 2: Interrupt for event PAYLOAD */
#define RADIO_INT_END                   (1 << 3)    /* Bit 3: Interrupt for event END */
#define RADIO_INT_DISABLED              (1 << 4)    /* Bit 4: Interrupt for event DISABLED */
#define RADIO_INT_DEVMATCH              (1 << 5)    /* Bit 5: Interrupt for event DEVMTCH */
#define RADIO_INT_DEVMISS               (1 << 6)    /* Bit 6: Interrupt for event DEVMISS */
#define RADIO_INT_RSSIEND               (1 << 7)    /* Bit 7: Interrupt for event RSSIEND */
#define RADIO_INT_BCMATCH               (1 << 10)   /* Bit 10: Interrupt for event BCMATCH */
#define RADIO_INT_CRCOK                 (1 << 12)   /* Bit 12: Interrupt for event CRCOK */
#define RADIO_INT_CRCERROR              (1 << 13)   /* Bit 13: Interrupt for event CRCERROR */
#define RADIO_INT_FRAMESTART            (1 << 14)   /* Bit 14: Interrupt for event FRAMESTART */
#define RADIO_INT_EDEND                 (1 << 15)   /* Bit 15: Interrupt for event EDEND */
#define RADIO_INT_EDSTOPPED             (1 << 16)   /* Bit 16: Interrupt for event EDSTOPPED */
#define RADIO_INT_CCAIDLE               (1 << 17)   /* Bit 17: Interrupt for event CCAIDLE */
#define RADIO_INT_CCABUSY               (1 << 18)   /* Bit 18: Interrupt for event CCABUSY */
#define RADIO_INT_CCASTOPPED            (1 << 19)   /* Bit 19: Interrupt for event CCASTOPPED */
#define RADIO_INT_RATEBOOST             (1 << 20)   /* Bit 20: Interrupt for event RATEBOOST */
#define RADIO_INT_TXREADY               (1 << 21)   /* Bit 21: Interrupt for event TXREADY */
#define RADIO_INT_RXREADY               (1 << 22)   /* Bit 22: Interrupt for event RXREADY */
#define RADIO_INT_MHRMATCH              (1 << 23)   /* Bit 23: Interrupt for event MHRMATCH */
#define RADIO_INT_PHYEND                (1 << 27)   /* Bit 27: Interrupt for event PHYEND */

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

#define RADIO_TXPOWER_SHIFT               (0)      /* Bits 0-7: RADIO output power */
#define RADIO_TXPOWER_MASK                (0xff << RADIO_TXPOWER_SHIFT)
#define RADIO_TXPOWER_MAX                 (0xff)

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
#  define RADIO_CRCCNF_LEN_1              (1 << RADIO_CRCCNF_LEN_SHIFTb)
#  define RADIO_CRCCNF_LEN_2              (2 << RADIO_CRCCNF_LEN_SHIFT)
#  define RADIO_CRCCNF_LEN_3              (3 << RADIO_CRCCNF_LEN_SHIFT)
#define RADIO_CRCCNF_SKIPADDR_SHIFT       (8)       /* Bit 8-9: Include or exclude packet address field out of CRC calculation */
#ifdef HAVE_RADIO_IEEE802154
#  define RADIO_CRCCNF_SKIPADDR_MASK      (0x3 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#  define RADIO_CRCCNF_SKIPADDR_INCL      (0 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#  define RADIO_CRCCNF_SKIPADDR_SKIP      (1 << RADIO_CRCCNF_SKIPADDR_SHIFT)
#  define RADIO_CRCCNF_SKIPADDR_IEEE      (3 << RADIO_CRCCNF_SKIPADDR_SHIFT)
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

#define RADIO_TIFS_SHIFT                  (0)      /* Bits 0-9: Interframe spacing in μs */
#define RADIO_TIFS_MASK                   (0x3ff << RADIO_TIFS_SHIFT)
#define RADIO_TIFS_MAX                    (0x3ff)

/* RSSISAMPLE Register */

#define RADIO_RSSISAMPLE_SHIFT            (0)     /* Bits 0-6: RSSI sample */
#define RADIO_RSSISAMPLE_MASK             (0x7f << RADIO_RSSISAMPLE_SHIFT)

/* STATE Register */

#define RADIO_STATE_SHIFT                 (0)     /* Bits 0-2: Current radio state */
#define RADIO_STATE_MASK                  (0xf << RADIO_STATE_SHIFT)
#  define RADIO_STATE_DISABLED            (0 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_RXRU                (1 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_RXIDLE              (2 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_RX                  (3 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_RXDISABLE           (4 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_TXRU                (9 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_TXIDLE              (10 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_TX                  (11 << RADIO_STATE_SHIFT)
#  define RADIO_STATE_TXDISABLE           (12 << RADIO_STATE_SHIFT)

/* DATAWHITEIV Register */

#define RADIO_DATAWHITEIV_SHIFT           (0)     /* Bits 0-6: Data whitening initial value */
#define RADIO_DATAWHITEIV_MASK            (0x3f << RADIO_DATAWHITEIV_SHIFT)
#define RADIO_DATAWHITEIV_MAX             (0x3f)

/* DAP Register */

#define RADIO_DAP_SHIFT                   (0)     /* Bits 0-15: Device address prefix n */
#define RADIO_DAP_MASK                    (0xffff << RADIO_DAP_SHIFT)

/* DANCF Register */

#define RADIO_DACNF_ENA(i)                (1 << (i))     /* Bits 0-7: Enable device address matching using */
#define RADIO_DACNF_TXADD(i)              (1 << (i + 8)) /* Bits 8-15: TXADD for device address i */

/* MODECNF0 Register */

#define RADIO_MODECNF0_RU                 (1 << 0) /* Bit 0: Radio ramp-up time */
#define RADIO_MODECNF0_DTX_SHIFT          (8)      /* Bits 8-9:  Default TX value */
#define RADIO_MODECNF0_DTX_MASK           (0x3 << RADIO_MODECNF0_DTX_SHIFT)
#  define RADIO_MODECNF0_DTX_B1           (0 << RADIO_MODECNF0_DTX_SHIFT)
#  define RADIO_MODECNF0_DTX_B0           (1 << RADIO_MODECNF0_DTX_SHIFT)
#  define RADIO_MODECNF0_DTX_CENTER       (2 << RADIO_MODECNF0_DTX_SHIFT)

#ifdef HAVE_RADIO_IEEE802154
/* SFD Register */

#  define RADIO_SFD_SHIFT                 (0)     /* Bits 0-7: IEEE 802.15.4 start of frame delimiter */
#  define RADIO_SFD_MASK                  (0xff << RADIO_SFD_SHIFT)

/* EDCNT Register */

#  define RADIO_EDCNT_SHIFT               (0)     /* Bits 0-20: IEEE 802.15.4 energy detect loop count */
#  define RADIO_EDCNT_MASK                (0x1fffff << RADIO_EDCNT_SHIFT)

/* EDSAMPLE Register */

#  define RADIO_EDSAMPLE_SHIFT            (0)     /* Bits 0-7: IEEE 802.15.4 energy detect level */
#  define RADIO_EDSAMPLE_MASK             (0xff << RADIO_EDSAMPLE_SHIFT)
#endif  /* CONFIG_NRF52_HAVE_RADIO_IEEE802154 */

/* CCACTRL Register */

#define RADIO_CCACTRL_CCAMODE_SHIFT       (0)     /* Bits 0-3: CCA mode of operation */
#define RADIO_CCACTRL_CCAMODE_MASK        (0x7 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_ED        (0 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_C         (1 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_CANDED    (2 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_CORED     (3 << RADIO_CCACTRL_CCAMODE_SHIFT)
#  define RADIO_CCACTRL_CCAMODE_EDTST1    (4 << RADIO_CCACTRL_CCAMODE_SHIFT)

/* POWER Register */

#define RADIO_POWER_ENABLE                (1 << 0) /* Bit 0: Peripheral power control */
#define RADIO_POWER_DISABLE               (0 << 0) /* Bit 0: Peripheral power control */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_RADIO_H */
