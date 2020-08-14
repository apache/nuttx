/****************************************************************************
 * arch/arm/src/samd5e5/sam_eic.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_EIC_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_EIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "sam_config.h"
#include "sam_port.h"
#include "hardware/sam_eic.h"

#ifdef CONFIG_SAMD5E5_EIC

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: sam_eic_initialize
 *
 * Description:
 *   Initialize the EIC.  Called one timer during system bring-up.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_initialize(void);

/****************************************************************************
 * Name: sam_eic_configure
 *
 * Description:
 *   Configure the interrupt edge sensitivity in CONFIGn register of the
 *   EIC.  The interrupt will be enabled at the EIC (but not at the NVIC).
 *
 * Input Parameters:
 *   eirq    - Pin to be configured (0..15)
 *   pinset  - Configuration of the pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_configure(uint8_t eirq, port_pinset_t pinset);

/****************************************************************************
 * Name: sam_eic_irq_ack
 *
 * Description:
 *   Acknowledge receipt of an external interrupt.
 *
 * Input Parameters:
 *   irq - SAM_IRQ_EXTINTn IRQ to be acknowledged, n=0-15
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_irq_ack(int irq);

void sam_eic_dumpregs(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* CONFIG_SAMD5E5_EIC */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_EIC_H */
