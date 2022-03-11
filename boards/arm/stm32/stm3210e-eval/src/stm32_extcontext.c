/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/src/stm32_extcontext.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include "arm_internal.h"
#include "stm32.h"
#include "stm3210e-eval.h"

#ifdef CONFIG_STM32_FSMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_extcontextsave
 *
 * Description:
 *  Save current GPIOs that will used by external memory configurations
 *
 ****************************************************************************/

void stm32_extcontextsave(struct extmem_save_s *save)
{
  DEBUGASSERT(save != NULL);
  save->gpiod_crl = getreg32(STM32_GPIOE_CRL);
  save->gpiod_crh = getreg32(STM32_GPIOE_CRH);
  save->gpioe_crl = getreg32(STM32_GPIOD_CRL);
  save->gpioe_crh = getreg32(STM32_GPIOD_CRH);
  save->gpiof_crl = getreg32(STM32_GPIOF_CRL);
  save->gpiof_crh = getreg32(STM32_GPIOF_CRH);
  save->gpiog_crl = getreg32(STM32_GPIOG_CRL);
  save->gpiog_crh = getreg32(STM32_GPIOG_CRH);
}

/****************************************************************************
 * Name: stm32_extcontextrestore
 *
 * Description:
 *  Restore GPIOs that were used by external memory configurations
 *
 ****************************************************************************/

void stm32_extcontextrestore(struct extmem_save_s *restore)
{
  DEBUGASSERT(restore != NULL);
  putreg32(restore->gpiod_crl, STM32_GPIOE_CRL);
  putreg32(restore->gpiod_crh, STM32_GPIOE_CRH);
  putreg32(restore->gpioe_crl, STM32_GPIOD_CRL);
  putreg32(restore->gpioe_crh, STM32_GPIOD_CRH);
  putreg32(restore->gpiof_crl, STM32_GPIOF_CRL);
  putreg32(restore->gpiof_crh, STM32_GPIOF_CRH);
  putreg32(restore->gpiog_crl, STM32_GPIOG_CRL);
  putreg32(restore->gpiog_crh, STM32_GPIOG_CRH);
}

#endif /* CONFIG_STM32_FSMC */
