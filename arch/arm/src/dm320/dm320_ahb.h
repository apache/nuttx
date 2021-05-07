/****************************************************************************
 * arch/arm/src/dm320/dm320_ahb.h
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

#ifndef __ARCH_ARM_SRC_DM320_DM320_AHB_H
#define __ARCH_ARM_SRC_DM320_DM320_AHB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AHB Bus Controller (AHBBUSC) Registers ***********************************/

#define DM320_AHB_SDRAMSA    (DM320_AHB_VADDR+0x0f00) /* SDRAM start address */
#define DM320_AHB_SDRAMEA    (DM320_AHB_VADDR+0x0f04) /* SDRAM end address */
#define DM320_AHB_BUSCONTROL (DM320_AHB_VADDR+0x0f08) /* Bus endianness control */
#define DM320_AHB_RSV1       (DM320_AHB_VADDR+0x0f0c) /* Reserved */
#define DM320_AHB_USBCTL     (DM320_AHB_VADDR+0x0f10) /* USB control register (ES1.1) */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_DM320_DM320_AHB_H */
