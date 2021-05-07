/****************************************************************************
 * arch/arm/src/xmc4/xmc4_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_XMC4_XMC4_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#define OFI_FREQUENCY 24000000 /* Frequency of internal Backup Clock Source */
#define OSI_FREQUENCY 32768    /* Frequency of internal Slow Clock Source */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_clock_configure
 *
 * Description:
 *   Called to initialize the XMC4xxx chip.  This does whatever setup is
 *   needed to put the  MCU in a usable state.  This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void xmc4_clock_configure(void);

/****************************************************************************
 * Name: xmc4_get_coreclock
 *
 * Description:
 *   Return the current core clock frequency, fCPU.
 *
 ****************************************************************************/

uint32_t xmc4_get_coreclock(void);

/****************************************************************************
 * Name: xmc4_get_periphclock
 *
 * Description:
 *   The peripheral clock is either fCPU or fCPU/2, depending on the state
 *   of the peripheral divider.
 *
 ****************************************************************************/

uint32_t xmc4_get_periphclock(void);

#endif /* __ARCH_ARM_SRC_XMC4_XMC4_CLOCKCONFIG_H */
