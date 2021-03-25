/****************************************************************************
 * arch/arm/src/dm320/dm320_busc.h
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

#ifndef __ARCH_ARM_SRC_DM320_DM320_BUSC_H
#define __ARCH_ARM_SRC_DM320_DM320_BUSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bus Controller Register Map (BUSC) ***************************************/

#define DM320_BUSC_ECR     (DM320_BUSC_REGISTER_BASE+0x0000) /* Endian Conversion Register */
#define DM320_BUSC_EBYTER  (DM320_BUSC_REGISTER_BASE+0x0002) /* Endian Byte Reverse Register */
#define DM320_BUSC_EBITR   (DM320_BUSC_REGISTER_BASE+0x0004) /* Endian Bit Reverse Register */
#define DM320_BUSC_REVR    (DM320_BUSC_REGISTER_BASE+0x0006) /* Device Revision Register */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_DM320_DM320_BUSC_H */
