/********************************************************************************************************************
 * arch/arm/src/xmc4/chip/xmc4_ethernet.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: XMC4500 Reference Manual V1.5 2014-07 Microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use with
 * Infineon's microcontrollers.  This file can be freely distributed within
 * development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ********************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_CHIP_XMC4_ETHERNET_H
#define __ARCH_ARM_SRC_XMC4_CHIP_XMC4_ETHERNET_H

/********************************************************************************************************************
 * Included Files
 ********************************************************************************************************************/

#include <nuttx/config.h>

#include "chip/xmc4_memorymap.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************************/

/* Register Offsets *************************************************************************************************/

/* MAC Configuration Registers */

#define XMC4_ETH_MAC_CONFIGURATION_OFFSET                    0x0000    /* MAC Configuration Register */
#define XMC4_ETH_MAC_FRAME_FILTER_OFFSET                     0x0004    /* MAC Frame Filter */
#define XMC4_ETH_HASH_TABLE_HIGH_OFFSET                      0x0008    /* Hash Table High Register */
#define XMC4_ETH_HASH_TABLE_LOW_OFFSET                       0x000c    /* Hash Table Low Register */
#define XMC4_ETH_GMII_ADDRESS_OFFSET                         0x0010    /* MII Address Register */
#define XMC4_ETH_GMII_DATA_OFFSET                            0x0014    /* MII Data Register */
#define XMC4_ETH_FLOW_CONTROL_OFFSET                         0x0018    /* Flow Control Register */
#define XMC4_ETH_VLAN_TAG_OFFSET                             0x001c    /* VLAN Tag Register */
#define XMC4_ETH_VERSION_OFFSET                              0x0020    /* Version Register */
#define XMC4_ETH_DEBUG_OFFSET                                0x0024    /* Debug Register */
#define XMC4_ETH_REMOTE_WAKE_UP_FRAME_FILTER_OFFSET          0x0028    /* Remote Wake Up Frame Filter Register */
#define XMC4_ETH_PMT_CONTROL_STATUS_OFFSET                   0x002c    /* PMT Control and Status Register */
#define XMC4_ETH_INTERRUPT_STATUS_OFFSET                     0x0038    /* Interrupt Register */
#define XMC4_ETH_INTERRUPT_MASK_OFFSET                       0x003c    /* Interrupt Mask Register */
#define XMC4_ETH_MAC_ADDRESS0_HIGH_OFFSET                    0x0040    /* MAC Address0 High Register */
#define XMC4_ETH_MAC_ADDRESS0_LOW_OFFSET                     0x0044    /* MAC Address0 Low Register */
#define XMC4_ETH_MAC_ADDRESS1_HIGH_OFFSET                    0x0048    /* MAC Address1 High Register */
#define XMC4_ETH_MAC_ADDRESS1_LOW_OFFSET                     0x004c    /* MAC Address1 Low Register */
#define XMC4_ETH_MAC_ADDRESS2_HIGH_OFFSET                    0x0050    /* MAC Address2 High Register */
#define XMC4_ETH_MAC_ADDRESS2_LOW_OFFSET                     0x0054    /* MAC Address2 Low Register */
#define XMC4_ETH_MAC_ADDRESS3_HIGH_OFFSET                    0x0058    /* MAC Address3 High Register */
#define XMC4_ETH_MAC_ADDRESS3_LOW_OFFSET                     0x005c    /* MAC Address3 Low Register */

/* MAC Management Counters */

#define XMC4_ETH_MMC_CONTROL_OFFSET                          0x0100    /* MMC Control Register */
#define XMC4_ETH_MMC_RECEIVE_INTERRUPT_OFFSET                0x0104    /* MMC Receive Interrupt Register */
#define XMC4_ETH_MMC_TRANSMIT_INTERRUPT_OFFSET               0x0108    /* MMC Transmit Interrupt Register */
#define XMC4_ETH_MMC_RECEIVE_INTERRUPT_MASK_OFFSET           0x010c    /* MMC Reveive Interrupt Mask Register */
#define XMC4_ETH_MMC_TRANSMIT_INTERRUPT_MASK_OFFSET          0x0110    /* MMC Transmit Interrupt Mask Register */
#define XMC4_ETH_TX_OCTET_COUNT_OFFSET                       0x0114    /* Transmit Octet Count for Good and Bad Frames Register */
#define XMC4_ETH_TX_FRAME_COUNT_OFFSET                       0x0118    /* Transmit Frame Count for Goodand Bad Frames Register */
#define XMC4_ETH_TX_BROADCAST_FRAMES_OFFSET                  0x011c    /* Transmit Frame Count for Good Broadcast Frames */
#define XMC4_ETH_TX_MULTICAST_FRAMES_OFFSET                  0x0120    /* Transmit Frame Count for Good Multicast Frames */
#define XMC4_ETH_TX_64OCTETS_FRAMES_OFFSET                   0x0124    /* Transmit Octet Count for Good and Bad 64 Byte Frames */
#define XMC4_ETH_TX_65TO127OCTETS_FRAMES_OFFSET              0x0128    /* Transmit Octet Count for Good and Bad 65 to 127 Bytes Frames */
#define XMC4_ETH_TX_128TO255OCTETS_FRAMES_OFFSET             0x012c    /* Transmit Octet Count for Good and Bad 128 to 255 Bytes Frames */
#define XMC4_ETH_TX_256TO511OCTETS_FRAMES_OFFSET             0x0130    /* Transmit Octet Count for Good and Bad 256 to 511 Bytes Frames */
#define XMC4_ETH_TX_512TO1023OCTETS_FRAMES_OFFSET            0x0134    /* Transmit Octet Count for Good and Bad 512 to 1023 Bytes Frames  */
#define XMC4_ETH_TX_1024TOMAXOCTETS_FRAMES_OFFSET            0x0138    /* Transmit Octet Count for Good and Bad 1024 to Maxsize Bytes Frames */
#define XMC4_ETH_TX_UNICAST_FRAMES_OFFSET                    0x013c    /* Transmit Frame Count for Good and Bad Unicast Frames */
#define XMC4_ETH_TX_MULTICAST_FRAMES_OFFSET                  0x0140    /* Transmit Frame Count for Good and Bad Multicast Frames */
#define XMC4_ETH_TX_BROADCAST_FRAMES_OFFSET                  0x0144    /* Transmit Frame Count for Good and Bad Broadcast Frames */
#define XMC4_ETH_TX_UNDERFLOW_ERROR_FRAMES_OFFSET            0x0148    /* Transmit Frame Count for Underflow Error Frames */
#define XMC4_ETH_TX_SINGLE_COLLISION_FRAMES_OFFSET           0x014c    /* Transmit Frame Count for Frames Transmitted after Single Collision */
#define XMC4_ETH_TX_MULTIPLE_COLLISION_FRAMES_OFFSET         0x0150    /* Transmit Frame Count for Frames Transmitted after Multiple Collision */
#define XMC4_ETH_TX_DEFERRED_FRAMES_OFFSET                   0x0154    /* Tx Deferred Frames Register */
#define XMC4_ETH_TX_LATE_COLLISION_FRAMES_OFFSET             0x0158    /* Transmit Frame Count for Late Collision Error Frames */
#define XMC4_ETH_TX_EXCESSIVE_COLLISION_FRAMES_OFFSET        0x015c    /* Transmit Frame Count for Excessive Collision Error Frames */
#define XMC4_ETH_TX_CARRIER_ERROR_FRAMES_OFFSET              0x0160    /* Transmit Frame Count for Carrier Sense Error Frames */
#define XMC4_ETH_TX_OCTET_COUNT_OFFSET                       0x0164    /* Tx Octet Count Good Register */
#define XMC4_ETH_TX_FRAME_COUNT_OFFSET                       0x0168    /* Tx Frame Count Good Register */
#define XMC4_ETH_TX_EXCESSIVE_DEFERRAL_ERROR_OFFSET          0x016c    /* Transmit Frame Count for Excessive Deferral Error Frames */
#define XMC4_ETH_TX_PAUSE_FRAMES_OFFSET                      0x0170    /* Transmit Frame Count for Good PAUSE Frames */
#define XMC4_ETH_TX_VLAN_FRAMES_OFFSET                       0x0174    /* Transmit Frame Count for Good VLAN Frames */
#define XMC4_ETH_TX_OSIZE_FRAMES_OFFSET                      0x0178    /* Transmit Frame Count for Good Oversize Frames */

#define XMC4_ETH_RX_FRAMES_COUNT_OFFSET                      0x0180    /* Receive Frame Count for Good and Bad Frames */
#define XMC4_ETH_RX_OCTET_COUNT_OFFSET                       0x0184    /* Receive Octet Count for Good and Bad Frames */
#define XMC4_ETH_RX_OCTET_COUNT_OFFSET                       0x0188    /* Rx Octet Count Good Register */
#define XMC4_ETH_RX_BROADCAST_FRAMES_OFFSET                  0x018c    /* Receive Frame Count for Good Broadcast Frames */
#define XMC4_ETH_RX_MULTICAST_FRAMES_OFFSET                  0x0190    /* Receive Frame Count for Good Multicast Frames */
#define XMC4_ETH_RX_CRC_ERROR_FRAMES_OFFSET                  0x0194    /* Receive Frame Count for CRC Error Frames */
#define XMC4_ETH_RX_ALIGNMENT_ERROR_FRAMES_OFFSET            0x0198    /* Receive Frame Count for Alignment Error Frames */
#define XMC4_ETH_RX_RUNT_ERROR_FRAMES_OFFSET                 0x019c    /* Receive Frame Count for Runt Error Frames */
#define XMC4_ETH_RX_JABBER_ERROR_FRAMES_OFFSET               0x01a0    /* Receive Frame Count for Jabber Error Frames */
#define XMC4_ETH_RX_UNDERSIZE_FRAMES_OFFSET                  0x01a4    /* Receive Frame Count for Undersize Frames */
#define XMC4_ETH_RX_OVERSIZE_FRAMES_OFFSET                   0x01a8    /* Rx Oversize Frames Good Register */
#define XMC4_ETH_RX_64OCTETS_FRAMES_OFFSET                   0x01ac    /* Receive Frame Count for Good and Bad 64 Byte Frames */
#define XMC4_ETH_RX_65TO127OCTETS_FRAMES_OFFSET              0x01b0    /* Receive Frame Count for Good and Bad 65 to 127 Bytes Frames */
#define XMC4_ETH_RX_128TO255OCTETS_FRAMES_OFFSET             0x01b4    /* Receive Frame Count for Good and Bad 128 to 255 Bytes Frames */
#define XMC4_ETH_RX_256TO511OCTETS_FRAMES_OFFSET             0x01b8    /* Receive Frame Count for Good and Bad 256 to 511 Bytes Frames */
#define XMC4_ETH_RX_512TO1023OCTETS_FRAMES_OFFSET            0x01bc    /* Receive Frame Count for Good and Bad 512 to 1,023 Bytes Frames */
#define XMC4_ETH_RX_1024TOMAXOCTETS_FRAMES_OFFSET            0x01c0    /* Receive Frame Count for Good and Bad 1,024 to Maxsize Bytes Frames */
#define XMC4_ETH_RX_UNICAST_FRAMES_OFFSET                    0x01c4    /* Receive Frame Count for Good Unicast Frames */
#define XMC4_ETH_RX_LENGTH_ERROR_FRAMES_OFFSET               0x01c8    /* Receive Frame Count for Length Error Frames */
#define XMC4_ETH_RX_OUT_OF_RANGE_TYPE_FRAMES_OFFSET          0x01cc    /* Receive Frame Count for Out of Range Frames */
#define XMC4_ETH_RX_PAUSE_FRAMES_OFFSET                      0x01d0    /* Receive Frame Count for PAUSE Frames */
#define XMC4_ETH_RX_FIFO_OVERFLOW_FRAMES_OFFSET              0x01d4    /* Receive Frame Count for FIFO Overflow Frames */
#define XMC4_ETH_RX_VLAN_FRAMES_OFFSET                       0x01d8    /* Receive Frame Count for Good and Bad VLAN Frames */
#define XMC4_ETH_RX_WATCHDOG_ERROR_FRAMES_OFFSET             0x01dc    /* Receive Frame Count for Watchdog Error Frames */
#define XMC4_ETH_RX_RECEIVE_ERROR_FRAMES_OFFSET              0x01e0    /* Receive Frame Count for Receive Error Frames */
#define XMC4_ETH_RX_CONTROL_FRAMES_OFFSET                    0x01e4    /* Receive Frame Count for Good Control Frames Frames */
#define XMC4_ETH_MMC_IPC_RECEIVE_INTERRUPT_MASK_OFFSET       0x0200    /* MMC Receive Checksum Offload Interrupt Mask Register */
#define XMC4_ETH_MMC_IPC_RECEIVE_INTERRUPT_OFFSET            0x0208    /* MMC Receive Checksum Offload Interrupt Register */
#define XMC4_ETH_RXIPV4_GOOD_FRAMES_OFFSET                   0x0210    /* RxIPv4 Good Frames Register */
#define XMC4_ETH_RXIPV4_HEADER_ERROR_FRAMES_OFFSET           0x0214    /* Receive IPV4 Header Error Frame Counter Register */
#define XMC4_ETH_RXIPV4_NO_PAYLOAD_FRAMES_OFFSET             0x0218    /* Receive IPV4 No Payload Frame Counter Register */
#define XMC4_ETH_RXIPV4_FRAGMENTED_FRAMES_OFFSET             0x021c    /* Receive IPV4 Fragmented Frame Counter Register */
#define XMC4_ETH_RXIPV4_UDP_CHECKSUM_DISABLED_FRAMES_OFFSET  0x0220    /* Receive IPV4 UDP Checksum Disabled Frame Counter Register */
#define XMC4_ETH_RXIPV6_GOOD_FRAMES_OFFSET                   0x0224    /* RxIPv6 Good Frames Register */
#define XMC4_ETH_RXIPV6_HEADER_ERROR_FRAMES_OFFSET           0x0228    /* Receive IPV6 Header Error Frame Counter Register */
#define XMC4_ETH_RXIPV6_NO_PAYLOAD_FRAMES_OFFSET             0x022c    /* Receive IPV6 No Payload Frame Counter Register */
#define XMC4_ETH_RXUDP_GOOD_FRAMES_OFFSET                    0x0230    /* RxUDP Good Frames Register */
#define XMC4_ETH_RXUDP_ERROR_FRAMES_OFFSET                   0x0234    /* RxUDP Error Frames Register */
#define XMC4_ETH_RXTCP_GOOD_FRAMES_OFFSET                    0x0238    /* RxTCP Good Frames Register */
#define XMC4_ETH_RXTCP_ERROR_FRAMES_OFFSET                   0x023c    /* RxTCP Error Frames Register */
#define XMC4_ETH_RXICMP_GOOD_FRAMES_OFFSET                   0x0240    /* RxICMP Good Frames Register */
#define XMC4_ETH_RXICMP_ERROR_FRAMES_OFFSET                  0x0244    /* RxICMP Error Frames Register */
#define XMC4_ETH_RXIPV4_GOOD_OCTETS_OFFSET                   0x0250    /* RxIPv4 Good Octets Register */
#define XMC4_ETH_RXIPV4_HEADER_ERROR_OCTETS_OFFSET           0x0254    /* Receive IPV4 Header Error Octet Counter Register */
#define XMC4_ETH_RXIPV4_NO_PAYLOAD_OCTETS_OFFSET             0x0258    /* Receive IPV4 No Payload Octet Counter Register */
#define XMC4_ETH_RXIPV4_FRAGMENTED_OCTETS_OFFSET             0x025c    /* Receive IPV4 Fragmented Octet Counter Register */
#define XMC4_ETH_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS_OFFSET   0x0260    /* Receive IPV4 Fragmented Octet Counter Register */
#define XMC4_ETH_RXIPV6_GOOD_OCTETS_OFFSET                   0x0264    /* RxIPv6 Good Octets Register */
#define XMC4_ETH_RXIPV6_HEADER_ERROR_OCTETS_OFFSET           0x0268    /* Receive IPV6 Header Error Octet Counter Register */
#define XMC4_ETH_RXIPV6_NO_PAYLOAD_OCTETS_OFFSET             0x026c    /* Receive IPV6 No Payload Octet Counter Register */
#define XMC4_ETH_RXUDP_GOOD_OCTETS_OFFSET                    0x0270    /* Receive UDP Good Octets Register */
#define XMC4_ETH_RXUDP_ERROR_OCTETS_OFFSET                   0x0274    /* Receive UDP Error Octets Register */
#define XMC4_ETH_RXTCP_GOOD_OCTETS_OFFSET                    0x0278    /* Receive TCP Good Octets Register */
#define XMC4_ETH_RXTCP_ERROR_OCTETS_OFFSET                   0x027c    /* Receive TCP Error Octets Register */
#define XMC4_ETH_RXICMP_GOOD_OCTETS_OFFSET                   0x0280    /* Receive ICMP Good Octets Register */
#define XMC4_ETH_RXICMP_ERROR_OCTETS_OFFSET                  0x0284    /* Receive ICMP Error Octets Register */

/* System Time Registers */

#define XMC4_ETH_TIMESTAMP_CONTROL_OFFSET                    0x0700    /* Timestamp Control Register */
#define XMC4_ETH_SUB_SECOND_INCREMENT_OFFSET                 0x0704    /* Sub-Second Increment Register */
#define XMC4_ETH_SYSTEM_TIME_SECONDS_OFFSET                  0x0708    /* System Time - Seconds Register */
#define XMC4_ETH_SYSTEM_TIME_NANOSECONDS_OFFSET              0x070c    /* System Time Nanoseconds Register */
#define XMC4_ETH_SYSTEM_TIME_SECONDS_UPDATE_OFFSET           0x0710    /* System Time - Seconds Update Register */
#define XMC4_ETH_SYSTEM_TIME_NANOSECONDS_UPDATE_OFFSET       0x0714    /* System Time Nanoseconds Update Register */
#define XMC4_ETH_TIMESTAMP_ADDEND_OFFSET                     0x0718    /* Timestamp Addend Register */
#define XMC4_ETH_TARGET_TIME_SECONDS_OFFSET                  0x071c    /* Target Time Seconds Register */
#define XMC4_ETH_TARGET_TIME_NANOSECONDS_OFFSET              0x0720    /* Target Time Nanoseconds Register */
#define XMC4_ETH_SYSTEM_TIME_HIGHER_WORD_SECONDS_OFFSET      0x0724    /* System Time - Higher Word Seconds Register */
#define XMC4_ETH_TIMESTAMP_STATUS_OFFSET                     0x0728    /* Timestamp Status Register */

/* DMA Registers*/

#define XMC4_ETH_BUS_MODE_OFFSET                             0x1000    /* Bus Mode Register */
#define XMC4_ETH_TRANSMIT_POLL_DEMAND_OFFSET                 0x1004    /* Transmit Poll Demand Register */
#define XMC4_ETH_RECEIVE_POLL_DEMAND_OFFSET                  0x1008    /* Receive Poll Demand Register */
#define XMC4_ETH_RECEIVE_DESCRIPTOR_LIST_ADDRESS_OFFSET      0x100c    /* Receive Descriptor Address Register */
#define XMC4_ETH_TRANSMIT_DESCRIPTOR_LIST_ADDRESS_OFFSET     0x1010    /* Transmit descripter Address Register */
#define XMC4_ETH_STATUS_OFFSET                               0x1014    /* Status Register */
#define XMC4_ETH_OPERATION_MODE_OFFSET                       0x1018    /* Operation Mode Register */
#define XMC4_ETH_INTERRUPT_ENABLE_OFFSET                     0x101c    /* Interrupt Enable Register */
#define XMC4_ETH_MISSED_FRAME_BUFFER_OVERFLOW_COUNTER_OFFSET 0x1020    /* Missed Frame and Buffer Overflow Counter Register */
#define XMC4_ETH_RECEIVE_INTERRUPT_WATCHDOG_TIMER_OFFSET     0x1024    /* Receive Interrupt Watchdog Timer Register */
#define XMC4_ETH_AHB_STATUS_OFFSET                           0x102c    /* AHB Status Register */
#define XMC4_ETH_CURRENT_HOST_TRANSMIT_DESCRIPTOR_OFFSET     0x1048    /* Current Host Transmit Descriptor Register */
#define XMC4_ETH_CURRENT_HOST_RECEIVE_DESCRIPTOR_OFFSET      0x104c    /* Current Host Receive Descriptor Register */
#define XMC4_ETH_CURRENT_HOST_TRANSMIT_BUFFER_ADDRESS_OFFSET 0x1050    /* Current Host Transmit Buffer Address Register */
#define XMC4_ETH_CURRENT_HOST_RECEIVE_BUFFER_ADDRESS_OFFSET  0x1054    /* Current Host Receive Buffer Address Register */
#define XMC4_ETH_HW_FEATURE_OFFSET                           0x1058    /* HW Feature Register */

/* Register Addresses ***********************************************************************************************/

/* Register Bit-Field Definitions ***********************************************************************************/


#endif /* __ARCH_ARM_SRC_XMC4_CHIP_XMC4_ETHERNET_H */
