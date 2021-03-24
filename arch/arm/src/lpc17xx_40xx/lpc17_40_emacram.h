/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_emacram.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_EMACRAM_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_EMACRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/lpc17_40_ethernet.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default, no-EMAC Case ****************************************************/

/* Assume that all of AHB SRAM will be available for heap. If this is not
 * true, then LPC17_40_BANK0_HEAPSIZE will be undefined and redefined below.
 */

#undef LPC17_40_BANK0_HEAPBASE
#undef LPC17_40_BANK0_HEAPSIZE
#ifdef LPC17_40_HAVE_BANK0
#  define LPC17_40_BANK0_HEAPBASE LPC17_40_SRAM_BANK0
#  define LPC17_40_BANK0_HEAPSIZE LPC17_40_BANK0_SIZE
#endif

/* Is networking enabled?
 * Is the LPC17xx/LPC40xx Ethernet device enabled? Does this chip have
 *  an Ethernet controller?
 *
 *  Yes... then we will replace the above default definitions.
 */

#if defined(CONFIG_NET) && defined(CONFIG_LPC17_40_ETHERNET) && LPC17_40_NETHCONTROLLERS > 0

/* EMAC RAM Configuration ***************************************************/

/* Is AHB SRAM available? */

#ifndef LPC17_40_HAVE_BANK0
#  error "AHB SRAM Bank0 is not available for EMAC RAM"
#endif

/* Number of Tx descriptors */

#ifndef CONFIG_LPC17_40_ETH_NTXDESC
#  define CONFIG_LPC17_40_ETH_NTXDESC 13
#endif

/* Number of Rx descriptors */

#ifndef CONFIG_LPC17_40_ETH_NRXDESC
#  define CONFIG_LPC17_40_ETH_NRXDESC 13
#endif

/* Size of the region at the beginning of AHB SRAM 0 set set aside for the
 * EMAC. This size must fit within AHB SRAM Bank 0 and also be a multiple
 * of 32-bit words.
 */

#ifndef CONFIG_LPC17_40_EMACRAM_SIZE
#  define CONFIG_LPC17_40_EMACRAM_SIZE LPC17_40_BANK0_SIZE
#endif

#if CONFIG_LPC17_40_EMACRAM_SIZE > LPC17_40_BANK0_SIZE
#  error "EMAC RAM size cannot exceed the size of AHB SRAM Bank 0"
#endif

#if (CONFIG_LPC17_40_EMACRAM_SIZE & 3) != 0
#  error "EMAC RAM size must be in multiples of 32-bit words"
#endif

/* Determine is there is any meaningful space left at the end of AHB Bank 0
 * that could be added to the heap.
 */

#undef LPC17_40_BANK0_HEAPBASE
#undef LPC17_40_BANK0_HEAPSIZE
#if CONFIG_LPC17_40_EMACRAM_SIZE < (LPC17_40_BANK0_SIZE-128)
#  define LPC17_40_BANK0_HEAPBASE (LPC17_40_SRAM_BANK0 + CONFIG_LPC17_40_EMACRAM_SIZE)
#  define LPC17_40_BANK0_HEAPSIZE (LPC17_40_BANK0_SIZE - CONFIG_LPC17_40_EMACRAM_SIZE)
#endif

/* Memory at the beginning of AHB SRAM, Bank 0 is set aside for EMAC Tx and
 * Rx descriptors.  The position is not controllable, only the size of the
 * region is controllable.
 */

#define LPC17_40_EMACRAM_BASE   LPC17_40_SRAM_BANK0
#define LPC17_40_EMACRAM_SIZE   CONFIG_LPC17_40_EMACRAM_SIZE

/* Descriptor Memory Layout *************************************************/

/* EMAC DMA RAM and descriptor definitions.  The configured number of
 * descriptors will determine the organization and the size of the descriptor
 * and status tables. There is a complex interaction between the maximum
 * packet size (CONFIG_NET_ETH_PKTSIZE) and the number of Rx and Tx
 * descriptors that can be supported (CONFIG_LPC17_40_ETH_NRXDESC and
 * CONFIG_LPC17_40_ETH_NTXDESC): Small buffers -> more packets.  This is
 * something that needs to be tuned for you system.
 *
 * For a 16Kb SRAM region, here is the relationship:
 *
 *  16384 <= ntx * (pktsize + 8 + 4) + nrx * (pktsize + 8 + 8)
 *
 * If ntx == nrx and pktsize == 590+2, then you could have ntx = nrx = 13.
 * In this case, you would need only 15,756 bytes of EMAC RAM (but be careful
 * with alignment! 15,756 is not well aligned.).
 */

#define LPC17_40_TXDESCTAB_SIZE (CONFIG_LPC17_40_ETH_NTXDESC*LPC17_40_TXDESC_SIZE)
#define LPC17_40_TXSTATTAB_SIZE (CONFIG_LPC17_40_ETH_NTXDESC*LPC17_40_TXSTAT_SIZE)
#define LPC17_40_TXTAB_SIZE     (LPC17_40_TXDESCTAB_SIZE+LPC17_40_TXSTATTAB_SIZE)

#define LPC17_40_RXDESCTAB_SIZE (CONFIG_LPC17_40_ETH_NRXDESC*LPC17_40_RXDESC_SIZE)
#define LPC17_40_RXSTATTAB_SIZE (CONFIG_LPC17_40_ETH_NRXDESC*LPC17_40_RXSTAT_SIZE)
#define LPC17_40_RXTAB_SIZE     (LPC17_40_RXDESCTAB_SIZE+LPC17_40_RXSTATTAB_SIZE)

#define LPC17_40_DESCTAB_SIZE   (LPC17_40_TXTAB_SIZE+LPC17_40_RXTAB_SIZE)

/* Descriptor table memory organization.  Descriptor tables are packed at
 * the end of AHB SRAM, Bank 0.  The beginning of bank 0 is reserved for
 * packet memory.
 */

#define LPC17_40_DESC_BASE      (LPC17_40_EMACRAM_BASE+LPC17_40_EMACRAM_SIZE-LPC17_40_DESCTAB_SIZE)
#define LPC17_40_TXDESC_BASE    LPC17_40_DESC_BASE
#define LPC17_40_TXSTAT_BASE    (LPC17_40_TXDESC_BASE+LPC17_40_TXDESCTAB_SIZE)
#define LPC17_40_RXDESC_BASE    (LPC17_40_TXSTAT_BASE+LPC17_40_TXSTATTAB_SIZE)
#define LPC17_40_RXSTAT_BASE    (LPC17_40_RXDESC_BASE + LPC17_40_RXDESCTAB_SIZE)

/* Now carve up the beginning of SRAM for packet memory.  The size of a
 * packet buffer is related to the size of the MTU.  We'll round sizes up
 * to multiples of 256 bytes.
 */

#define LPC17_40_PKTMEM_BASE     LPC17_40_EMACRAM_BASE
#define LPC17_40_PKTMEM_SIZE     (LPC17_40_EMACRAM_SIZE-LPC17_40_DESCTAB_SIZE)
#define LPC17_40_PKTMEM_END      (LPC17_40_EMACRAM_BASE+LPC17_40_PKTMEM_SIZE)

#define LPC17_40_MAXPACKET_SIZE  ((CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE + 3) & ~3)
#define LPC17_40_NTXPKTS         CONFIG_LPC17_40_ETH_NTXDESC
#define LPC17_40_NRXPKTS         CONFIG_LPC17_40_ETH_NRXDESC

#define LPC17_40_TXBUFFER_SIZE   (LPC17_40_NTXPKTS * LPC17_40_MAXPACKET_SIZE)
#define LPC17_40_RXBUFFER_SIZE   (LPC17_40_NRXPKTS * LPC17_40_MAXPACKET_SIZE)
#define LPC17_40_BUFFER_SIZE     (LPC17_40_TXBUFFER_SIZE + LPC17_40_RXBUFFER_SIZE)

#define LPC17_40_BUFFER_BASE     LPC17_40_PKTMEM_BASE
#define LPC17_40_TXBUFFER_BASE   LPC17_40_BUFFER_BASE
#define LPC17_40_RXBUFFER_BASE   (LPC17_40_TXBUFFER_BASE + LPC17_40_TXBUFFER_SIZE)
#define LPC17_40_BUFFER_END      (LPC17_40_BUFFER_BASE + LPC17_40_BUFFER_SIZE)

#if LPC17_40_BUFFER_END > LPC17_40_PKTMEM_END
#  error "Packet memory overlaps descriptor tables"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_NET && CONFIG_LPC17_40_ETHERNET && LPC17_40_NETHCONTROLLERS > 0*/
#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_EMACRAM_H */
