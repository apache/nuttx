/****************************************************************************
 * arch/arm/src/sama5/sam_irq.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_IRQ_H
#define __ARCH_ARM_SRC_SAMA5_SAM_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_aic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCRTYPE_NTYPES    6
#define SAM_DEFAULT_PRIOR ((AIC_SMR_PRIOR_MAX+1) >> 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum sam_srctype_e
{
  SRCTYPE_IHIGH    = 0, /* Internal high level */
  SRCTYPE_XLOW     = 1, /* External low level */
  SRCTYPE_IRISING  = 2, /* Internal positive edge */
  SRCTYPE_XFALLING = 3, /* External negative edge */
  SRCTYPE_XHIGH    = 4, /* External high level */
  SRCTYPE_XRISING  = 5  /* External rising edge */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_irq_srctype
 *
 * Description:
 *   irq     - Identifies the IRQ source to be configured
 *   srctype - IRQ source configuration
 *
 ****************************************************************************/

void sam_irq_srctype(int irq, enum sam_srctype_e srctype);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_IRQ_H */
