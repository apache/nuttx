/****************************************************************************
 * arch/arm/src/xmc4/xmc4_usic.h
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_USIC_H
#define __ARCH_ARM_SRC_XMC4_XMC4_USIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "xmc4_config.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enumeration identifies the USIC */

enum usic_e
{
  USIC0       = 0,    /* USIC0 */
  USIC1       = 1,    /* USIC1 */
  USIC2       = 2     /* USIC2 */
};

/* This enumeration identifies USIC channels */

enum usic_channel_e
{
  USIC0_CHAN0 = 0,    /* USIC0, Channel 0 */
  USIC0_CHAN1 = 1,    /* USIC0, Channel 1 */
  USIC1_CHAN0 = 2,    /* USIC1, Channel 0 */
  USIC1_CHAN1 = 3,    /* USIC1, Channel 1 */
  USIC2_CHAN0 = 4,    /* USIC2, Channel 0 */
  USIC2_CHAN1 = 5     /* USIC2, Channel 1 */
};

/* This enumeration defines values for the dx input selection */

enum uart_dx_e
{
  USIC_DXA    = 0,    /* USICn_DXmA */
  USIC_DXB    = 1,    /* USICn_DXmB */
  USIC_DXC    = 2,    /* USICn_DXmC */
  USIC_DXD    = 3,    /* USICn_DXmD */
  USIC_DXE    = 4,    /* USICn_DXmE */
  USIC_DXF    = 5,    /* USICn_DXmF */
  USIC_DXG    = 6     /* USICn_DXmG */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_enable_usic
 *
 * Description:
 *   Enable the USIC module indicated by the 'usic' enumeration value
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_enable_usic(enum usic_e usic);

/****************************************************************************
 * Name: xmc4_disable_usic
 *
 * Description:
 *   Disable the USIC module indicated by the 'usic' enumeration value
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_disable_usic(enum usic_e usic);

/****************************************************************************
 * Name: xmc4_channel2usic
 *
 * Description:
 *   Given a USIC channel enumeration value, return the corresponding USIC
 *   enumerication value.
 *
 * Returned Value:
 *   The corresponding USIC enumeration value.
 *
 ****************************************************************************/

static inline enum usic_e xmc4_channel2usic(enum usic_channel_e channel)
{
  return (enum usic_e)((unsigned int)channel >> 1);
}

/****************************************************************************
 * Name: xmc4_channel_baseaddress
 *
 * Description:
 *   Given a USIC channel enumeration value, return the base address of the
 *   channel registers.
 *
 * Returned Value:
 *   The non-zero address of the channel base registers is return on success.
 *   Zero is returned on any failure.
 *
 ****************************************************************************/

uintptr_t xmc4_channel_baseaddress(enum usic_channel_e channel);

/****************************************************************************
 * Name: xmc4_enable_usic_channel
 *
 * Description:
 *   Enable the USIC channel indicated by 'channel'.  Also enable and reset
 *   the USIC module if it is not already enabled.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_enable_usic_channel(enum usic_channel_e channel);

/****************************************************************************
 * Name: xmc4_disable_usic_channel
 *
 * Description:
 *   Disable the USIC channel indicated by 'channel'.  Also disable and reset
 *   the USIC module if both channels have been disabled.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_disable_usic_channel(enum usic_channel_e channel);

/****************************************************************************
 * Name: xmc4_usic_baudrate
 *
 * Description:
 *   Set the USIC baudrate for the USIC channel
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_usic_baudrate(enum usic_channel_e channel, uint32_t baud,
                       uint32_t oversampling);

#endif /* __ARCH_ARM_SRC_XMC4_XMC4_USIC_H */
