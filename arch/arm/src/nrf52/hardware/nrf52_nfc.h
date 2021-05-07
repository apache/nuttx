/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_nfc.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_NFC_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_NFC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF52_NFC_TASKS_ACTIVATE_OFFSET       0x0000 /* Activate NFCT peripheral */
#define NRF52_NFC_TASKS_DISABLE_OFFSET        0x0004 /* Disable NFCT peripheral */
#define NRF52_NFC_TASKS_SENSE_OFFSET          0x0008 /* Enable NFC sense field mode */
#define NRF52_NFC_TASKS_STARTTX_OFFSET        0x000c /* Start transmission of an outgoing frame */
#define NRF52_NFC_TASKS_ENABLERXDATA_OFFSET   0x001c /* Initializes the EasyDMA for receive */
#define NRF52_NFC_TASKS_GOIDLE_OFFSET         0x0024 /* Force state machine to IDLE state */
#define NRF52_NFC_TASKS_GOSLEEP_OFFSET        0x0028 /* Force state machine to SLEEP_A state */
#define NRF52_NFC_EVENTS_READY_OFFSET         0x0100 /* The NFCT peripheral is ready to receive and send frames */
#define NRF52_NFC_EVENTS_FIELDDETECTED_OFFSET 0x0104 /* Remote NFC field detected */
#define NRF52_NFC_EVENTS_FIELDLOST_OFFSET     0x0108 /* Remote NFC field lost */
#define NRF52_NFC_EVENTS_TXFRAMESTART_OFFSET  0x010c /* Marks the start of the first symbol of a transmitted frame */
#define NRF52_NFC_EVENTS_TXFRAMEEND_OFFSET    0x0110 /* Marks the end of the last transmitted on-air symbol of a frame */
#define NRF52_NFC_EVENTS_RXFRAMESTART_OFFSET  0x0114 /* Marks the end of the first symbol of a received frame */
#define NRF52_NFC_EVENTS_RXFRAMEEND_OFFSET    0x0118 /* Received data has been checked (CRC, parity) and transferred to RAM */
#define NRF52_NFC_EVENTS_ERROR_OFFSET         0x011c /* NFC error reported */
#define NRF52_NFC_EVENTS_RXERROR_OFFSET       0x0128 /* NFC RX frame error reported */
#define NRF52_NFC_EVENTS_ENDRX_OFFSET         0x012c /* RX buffer in Data RAM full */
#define NRF52_NFC_EVENTS_ENDTX_OFFSET         0x0130 /* Transmission of data in RAM has ended */
#define NRF52_NFC_EVENTS_ACRSTARTED_OFFSET    0x0138 /* Auto collision resolution process has started */
#define NRF52_NFC_EVENTS_COLLISION_OFFSET     0x0148 /* NFC auto collision resolution error reported */
#define NRF52_NFC_EVENTS_SELECTED_OFFSET      0x014c /* NFC auto collision resolution successfully completed */
#define NRF52_NFC_EVENTS_STARTED_OFFSET       0x0150 /* EasyDMA is ready to receive or send frames */
#define NRF52_NFC_SHORTS_OFFSET               0x0200 /* Shortcuts between local events and tasks */
#define NRF52_NFC_INTEN_OFFSET                0x0300 /* Enable or disable interrupt */
#define NRF52_NFC_INTENSET_OFFSET             0x0304 /* Enable interrupt */
#define NRF52_NFC_INTENCLR_OFFSET             0x0308 /* Disable interrupt */
#define NRF52_NFC_ERRORSTATUS_OFFSET          0x0404 /* NFC Error Status register */
#define NRF52_NFC_FRAMESTATUSRX_OFFSET        0x040c /* Result of last incoming frame */
#define NRF52_NFC_NFCTAGSTATE_OFFSET          0x0410 /* NFC Tag state register */
#define NRF52_NFC_SLEEPSTATE_OFFSET           0x0420 /* Sleep state during automatic collision resolution */
#define NRF52_NFC_FIELDPRESENT_OFFSET         0x043c /* Indicates the presence or not of a valid field */
#define NRF52_NFC_FRAMEDELAYMIN_OFFSET        0x0504 /* Minimum frame delay */
#define NRF52_NFC_FRAMEDELAYMAX_OFFSET        0x0508 /* Maximum frame delay */
#define NRF52_NFC_FRAMEDELAYMODE_OFFSET       0x050c /* Configuration register for the Frame Delay Timer */
#define NRF52_NFC_PACKETPTR_OFFSET            0x0510 /* Packet pointer for TXD and RXD data storage in Data RAM */
#define NRF52_NFC_MAXLEN_OFFSET               0x0514 /* Size of the RAM buffer allocated to TXD and RXD data storage each */
#define NRF52_NFC_TXDFRAMECONFIG_OFFSET       0x0518 /* Configuration of outgoing frames */
#define NRF52_NFC_TXDAMOUNT_OFFSET            0x051c /* Size of outgoing frame */
#define NRF52_NFC_RXDFRAMECONFIG_OFFSET       0x0520 /* Configuration of incoming frames */
#define NRF52_NFC_RXDAMOUNT_OFFSET            0x0524 /* Size of last incoming frame */
#define NRF52_NFC_NFCID1LAST_OFFSET           0x0590 /* Last NFCID part (4, 7 or 10 bytes ID) */
#define NRF52_NFC_NFCID2LAST_OFFSET           0x0594 /* Second last NFCID part (7 or 10 bytes ID) */
#define NRF52_NFC_NFCID3LAST_OFFSET           0x0598 /* Third last NFCID part (10 bytes ID) */
#define NRF52_NFC_ACRCONFIG_OFFSET            0x059c /* Controls the auto collision resolution function */
#define NRF52_NFC_SENSRES_OFFSET              0x05a0 /* NFC-A SENS_RES auto-response settings */
#define NRF52_NFC_SELRES_OFFSET               0x05a4 /* NFC-A SEL_RES auto-response settings */

/* Register Addresses *******************************************************/

#define NRF52_NFC_TASKS_ACTIVATE       (NRF52_NFCT_BASE + NRF52_NFC_TASKS_ACTIVATE_OFFSET)
#define NRF52_NFC_TASKS_DISABLE        (NRF52_NFCT_BASE + NRF52_NFC_TASKS_DISABLE_OFFSET)
#define NRF52_NFC_TASKS_SENSE          (NRF52_NFCT_BASE + NRF52_NFC_TASKS_SENSE_OFFSET)
#define NRF52_NFC_TASKS_STARTTX        (NRF52_NFCT_BASE + NRF52_NFC_TASKS_STARTTX_OFFSET)
#define NRF52_NFC_TASKS_ENABLERXDATA   (NRF52_NFCT_BASE + NRF52_NFC_TASKS_ENABLERXDATA_OFFSET)
#define NRF52_NFC_TASKS_GOIDLE         (NRF52_NFCT_BASE + NRF52_NFC_TASKS_GOIDLE_OFFSET)
#define NRF52_NFC_TASKS_GOSLEEP        (NRF52_NFCT_BASE + NRF52_NFC_TASKS_GOSLEEP_OFFSET)
#define NRF52_NFC_EVENTS_READY         (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_READY_OFFSET)
#define NRF52_NFC_EVENTS_FIELDDETECTED (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_FIELDDETECTED_OFFSET)
#define NRF52_NFC_EVENTS_FIELDLOST     (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_FIELDLOST_OFFSET)
#define NRF52_NFC_EVENTS_TXFRAMESTART  (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_TXFRAMESTART_OFFSET)
#define NRF52_NFC_EVENTS_TXFRAMEEND    (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_TXFRAMEEND_OFFSET)
#define NRF52_NFC_EVENTS_RXFRAMESTART  (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_RXFRAMESTART_OFFSET)
#define NRF52_NFC_EVENTS_RXFRAMEEND    (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_RXFRAMEEND_OFFSET)
#define NRF52_NFC_EVENTS_ERROR         (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_ERROR_OFFSET)
#define NRF52_NFC_EVENTS_RXERROR       (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_RXERROR_OFFSET)
#define NRF52_NFC_EVENTS_ENDRX         (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_ENDRX_OFFSET)
#define NRF52_NFC_EVENTS_ENDTX         (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_ENDTX_OFFSET)
#define NRF52_NFC_EVENTS_ACRSTARTED    (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_ACRSTARTED_OFFSET)
#define NRF52_NFC_EVENTS_COLLISION     (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_COLLISION_OFFSET)
#define NRF52_NFC_EVENTS_SELECTED      (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_SELECTED_OFFSET)
#define NRF52_NFC_EVENTS_STARTED       (NRF52_NFCT_BASE + NRF52_NFC_EVENTS_STARTED_OFFSET)
#define NRF52_NFC_SHORTS               (NRF52_NFCT_BASE + NRF52_NFC_SHORTS_OFFSET)
#define NRF52_NFC_INTEN                (NRF52_NFCT_BASE + NRF52_NFC_INTEN_OFFSET)
#define NRF52_NFC_INTENSET             (NRF52_NFCT_BASE + NRF52_NFC_INTENSET_OFFSET)
#define NRF52_NFC_INTENCLR             (NRF52_NFCT_BASE + NRF52_NFC_INTENCLR_OFFSET)
#define NRF52_NFC_ERRORSTATUS          (NRF52_NFCT_BASE + NRF52_NFC_ERRORSTATUS_OFFSET)
#define NRF52_NFC_FRAMESTATUSRX        (NRF52_NFCT_BASE + NRF52_NFC_FRAMESTATUSRX_OFFSET)
#define NRF52_NFC_NFCTAGSTATE          (NRF52_NFCT_BASE + NRF52_NFC_NFCTAGSTATE_OFFSET)
#define NRF52_NFC_SLEEPSTATE           (NRF52_NFCT_BASE + NRF52_NFC_SLEEPSTATE_OFFSET)
#define NRF52_NFC_FIELDPRESENT         (NRF52_NFCT_BASE + NRF52_NFC_FIELDPRESENT_OFFSET)
#define NRF52_NFC_FRAMEDELAYMIN        (NRF52_NFCT_BASE + NRF52_NFC_FRAMEDELAYMIN_OFFSET)
#define NRF52_NFC_FRAMEDELAYMAX        (NRF52_NFCT_BASE + NRF52_NFC_FRAMEDELAYMAX_OFFSET)
#define NRF52_NFC_FRAMEDELAYMODE       (NRF52_NFCT_BASE + NRF52_NFC_FRAMEDELAYMODE_OFFSET)
#define NRF52_NFC_PACKETPTR            (NRF52_NFCT_BASE + NRF52_NFC_PACKETPTR_OFFSET)
#define NRF52_NFC_MAXLEN               (NRF52_NFCT_BASE + NRF52_NFC_MAXLEN_OFFSET)
#define NRF52_NFC_TXDFRAMECONFIG       (NRF52_NFCT_BASE + NRF52_NFC_TXDFRAMECONFIG_OFFSET)
#define NRF52_NFC_TXDAMOUNT            (NRF52_NFCT_BASE + NRF52_NFC_TXDAMOUNT_OFFSET)
#define NRF52_NFC_RXDFRAMECONFIG       (NRF52_NFCT_BASE + NRF52_NFC_RXDFRAMECONFIG_OFFSET)
#define NRF52_NFC_RXDAMOUNT            (NRF52_NFCT_BASE + NRF52_NFC_RXDAMOUNT_OFFSET)
#define NRF52_NFC_NFCID1LAST           (NRF52_NFCT_BASE + NRF52_NFC_NFCID1LAST_OFFSET)
#define NRF52_NFC_NFCID2LAST           (NRF52_NFCT_BASE + NRF52_NFC_NFCID2LAST_OFFSET)
#define NRF52_NFC_NFCID3LAST           (NRF52_NFCT_BASE + NRF52_NFC_NFCID3LAST_OFFSET)
#define NRF52_NFC_ACRCONFIG            (NRF52_NFCT_BASE + NRF52_NFC_ACRCONFIG_OFFSET)
#define NRF52_NFC_SENSRES              (NRF52_NFCT_BASE + NRF52_NFC_SENSRES_OFFSET)
#define NRF52_NFC_SELRES               (NRF52_NFCT_BASE + NRF52_NFC_SELRESOFFSET)

/* Register Bitfield Definitions ********************************************/

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_NFC_H */
