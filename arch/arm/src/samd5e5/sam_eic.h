/****************************************************************************
 * arch/arm/src/samd5e5/sam_eic.h
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
