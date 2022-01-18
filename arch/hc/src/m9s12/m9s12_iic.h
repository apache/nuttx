/****************************************************************************
 * arch/hc/src/m9s12/m9s12_iic.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_IIC_H
#define __ARCH_HC_SRC_M9S12_M9S12_IIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define HCS12_IIC_IBAD_OFFSET          0x0000 /* IIC Address Register */
#define HCS12_IIC_IBFD_OFFSET          0x0001 /* IIC Frequency Divider Register */
#define HCS12_IIC_IBCR_OFFSET          0x0002 /* IIC Control Register */
#define HCS12_IIC_IBSR_OFFSET          0x0003 /* IIC Status Register */
#define HCS12_IIC_IBDR_OFFSET          0x0004 /* IIC Data I/O Register */

/* Register Addresses *******************************************************/

#define HCS12_IIC_IBAD                 (HCS12_IIC_BASE+HCS12_IIC_IBAD_OFFSET)
#define HCS12_IIC_IBFD                 (HCS12_IIC_BASE+HCS12_IIC_IBFD_OFFSET)
#define HCS12_IIC_IBCR                 (HCS12_IIC_BASE+HCS12_IIC_IBCR_OFFSET)
#define HCS12_IIC_IBSR                 (HCS12_IIC_BASE+HCS12_IIC_IBSR_OFFSET)
#define HCS12_IIC_IBDR                 (HCS12_IIC_BASE+HCS12_IIC_IBDR_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* IIC Address Register */

#define IIC_IBAD_MASK                  (0xfe)

/* IIC Frequency Divider Register -- 8-bit bus clock rate value */

/* IIC Control Register */

#define IIC_IBCR_IBSWAI                (1 << 0)  /* Bit 0: I Bus Interface Stop in Wait Mode */
#define IIC_IBCR_RSTA                  (1 << 2)  /* Bit 2: Repeat Start */
#define IIC_IBCR_TXAK                  (1 << 3)  /* Bit 3: Transmit Acknowledge Enable */
#define IIC_IBCR_TX                    (1 << 4)  /* Bit 4: Transmit/Receive Mode Select Bit */
#define IIC_IBCR_MSSL                  (1 << 5)  /* Bit 5: Master/Slave Mode Select Bit */
#define IIC_IBCR_IBIE                  (1 << 6)  /* Bit 6: I-Bus Interrupt Enable */
#define IIC_IBCR_IBEN                  (1 << 7)  /* Bit 7: I-Bus Enable */

/* IIC Status Register */

#define IIC_IBSR_RXAK                  (1 << 0)  /* Bit 0: Received Acknowledge */
#define IIC_IBSR_IBIF                  (1 << 1)  /* Bit 1: I-Bus Interrupt */
#define IIC_IBSR_SRW                   (1 << 2)  /* Bit 2: Slave Read/Write */
#define IIC_IBSR_AL                    (1 << 4)  /* Bit 4: Arbitration Lost */
#define IIC_IBSR_BB                    (1 << 5)  /* Bit 5: Bus Busy Bit */
#define IIC_IBSR_AAS                   (1 << 6)  /* Bit 6: Addressed as a Slave Bit */
#define IIC_IBSR_TCF                   (1 << 7)  /* Bit 7: Data Transferring Bit */

/* IIC Data I/O Register -- 8-Bit data value */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_M9S12_IIC_H */
