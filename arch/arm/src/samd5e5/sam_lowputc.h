/****************************************************************************
 * arch/arm/src/samd5e5/sam_lowputc.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_LOWPUTC_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "sam_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization.
 *
 ****************************************************************************/

void sam_lowsetup(void);

/****************************************************************************
 * Name: sam_usart_initialize
 *
 * Description:
 *   Set the configuration of a SERCOM for provided USART configuration.
 *   This configures the SERCOM as a USART, but does not configure USART
 *   interrupts or enable the USART.
 *
 ****************************************************************************/

#ifdef SAMD5E5_HAVE_USART
struct sam_usart_config_s;
int sam_usart_initialize(const struct sam_usart_config_s * const config);
#endif

/****************************************************************************
 * Name: sam_usart_reset
 *
 * Description:
 *   Reset the USART SERCOM.  This restores all SERCOM register to the
 *   initial state and disables the SERCOM.
 *
 ****************************************************************************/

#ifdef SAMD5E5_HAVE_USART
struct sam_usart_config_s;
void sam_usart_reset(const struct sam_usart_config_s * const config);
#endif

/****************************************************************************
 * Name: sam_lowputc
 *
 * Description:
 *   Output one character to the USART using a simple polling method.
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void sam_lowputc(uint32_t ch);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_LOWPUTC_H */
