/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_lowputc.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_LOWPUTC_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
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
 * Name: gd32_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.
 *   Performs low level initialization of serial console.
 *
 ****************************************************************************/

void gd32_lowsetup(void);

/****************************************************************************
 * Name: gd32_usart_reset
 *
 * Description:
 *   Reset the USART.
 *
 ****************************************************************************/

void gd32_usart_reset(uint32_t usartbase);

/****************************************************************************
 * Name: gd32_usart_clock_enable
 *
 * Description:
 *   Enable USART clock
 ****************************************************************************/

void gd32_usart_clock_enable(uint32_t usartbase);

/****************************************************************************
 * Name: gd32_usart_clock_disable
 *
 * Description:
 *   Dinable USART clock
 ****************************************************************************/

void gd32_usart_clock_disable(uint32_t usartbase);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_LOWPUTC_H */
