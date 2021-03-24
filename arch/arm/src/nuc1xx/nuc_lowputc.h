/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_lowputc.h
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

#ifndef __ARCH_ARM_SRC_NUC1XX_NUC_LOWPUTC_H
#define __ARCH_ARM_SRC_NUC1XX_NUC_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nuc_config.h"

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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nuc_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.
 *   Performs low level initialization.
 *
 ****************************************************************************/

void nuc_lowsetup(void);

/****************************************************************************
 * Name: nuc_setbaud
 *
 * Description:
 *   Set the BAUD divxisor for the selected UART
 *
 *   Mode DIV_X_EN DIV_X_ONE Divider X   BRD  (Baud rate equation)
 *   -------------------------------------------------------------
 *   0    Disable  0         B           A    UART_CLK / [16 * (A+2)]
 *   1    Enable   0         B           A    UART_CLK / [(B+1) * (A+2)] ,
 *                                                 B must >= 8
 *   2    Enable   1         Don't care  A    UART_CLK / (A+2), A must >=3
 *
 * Here we assume that the default clock source for the UART modules is
 * the external high speed crystal.
 *
 ****************************************************************************/

#ifdef HAVE_UART
void nuc_setbaud(uintptr_t base, uint32_t baud);
#endif

/****************************************************************************
 * Name: nuc_lowputc
 *
 * Description:
 *   Output one character to the UART using a simple polling method.
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void nuc_lowputc(uint32_t ch);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_NUC1XX_NUC_LOWPUTC_H */
