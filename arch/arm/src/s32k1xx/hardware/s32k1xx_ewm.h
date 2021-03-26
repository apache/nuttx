/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_ewm.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_EWM_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_EWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EWM Register Offsets *****************************************************/

#define S32K1XX_EWM_CTRL_OFFSET          0x0000  /* Control Register */
#define S32K1XX_EWM_SERV_OFFSET          0x0001  /* Service Register */
#define S32K1XX_EWM_CMPL_OFFSET          0x0002  /* Compare Low Register */
#define S32K1XX_EWM_CMPH_OFFSET          0x0003  /* Compare High Register */
#define S32K1XX_EWM_CLKPRESCALER_OFFSET  0x0005  /* Clock Prescaler Register */

/* EWM Register Addresses ***************************************************/

#define S32K1XX_EWM_CTRL                 (S32K1XX_EWM_BASE + S32K1XX_EWM_CTRL_OFFSET)
#define S32K1XX_EWM_SERV                 (S32K1XX_EWM_BASE + S32K1XX_EWM_SERV_OFFSET)
#define S32K1XX_EWM_CMPL                 (S32K1XX_EWM_BASE + S32K1XX_EWM_CMPL_OFFSET)
#define S32K1XX_EWM_CMPH                 (S32K1XX_EWM_BASE + S32K1XX_EWM_CMPH_OFFSET)
#define S32K1XX_EWM_CLKPRESCALER         (S32K1XX_EWM_BASE + S32K1XX_EWM_CLKPRESCALER_OFFSET)

/* EWM Register Bitfield Definitions ****************************************/

/* Control Register */

#define EWM_CTRL_EWMEN                   (1 << 0)  /* Bit 0:  EWM enable */
#define EWM_CTRL_ASSIN                   (1 << 1)  /* Bit 1:  EWM_in's assertion state select */
#define EWM_CTRL_INEN                    (1 << 2)  /* Bit 2:  Input enable */
#define EWM_CTRL_INTEN                   (1 << 3)  /* Bit 3:  Interrupt enable */

/* Service Register (8-bit SERVICE value) */

#define EWM_SERV_BYTE1                   0xb4
#define EWM_SERV_BYTE1                   0x2c

/* Compare Low Register (8-bit COMPAREL value) */

/* Compare High Register (8-bit COMPAREH value) */

/* Clock Prescaler Register (8-bit CLK_DIV value) */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_EWM_H */
