/****************************************************************************
 * arch/arm/src/stm32/stm32_foc.h
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

#ifndef __ARCH_ARM_SRC_STM32_FOC_H
#define __ARCH_ARM_SRC_STM32_FOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32_adc.h"

#include <nuttx/motor/foc/foc_lower.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ADC configuration for the FOC device */

struct stm32_foc_adc_s
{
  /* ADC interface used by the FOC */

  uint8_t intf;

  /* The number of ADC channels (regular + injected) */

  uint8_t nchan;

  /* The number of auxiliary regular channels (only for DMA transfer) */

  uint8_t regch;

  /* The list of ADC channels (regular first, then injected) */

  FAR uint8_t *chan;

  /* The list of ADC pins */

  FAR uint32_t *pins;

  /* The list of ADC channels sample time configuration */

  FAR adc_channel_t *stime;
};

/* Board-specific operations.
 *
 * These are calls from the lower-half to the board-specific logic.
 * They must be provided by board-specific logic even if not used.
 */

struct stm32_foc_board_ops_s
{
  /* Board-specific setup */

  CODE int (*setup)(FAR struct foc_dev_s *dev);

  /* Board-specific shutdown */

  CODE int (*shutdown)(FAR struct foc_dev_s *dev);

  /* Board-specific calibration setup */

  CODE int (*calibration)(FAR struct foc_dev_s *dev, bool state);

  /* Board-specific fault clear */

  CODE int (*fault_clear)(FAR struct foc_dev_s *dev);

  /* Board-specific PWM start */

  CODE int (*pwm_start)(FAR struct foc_dev_s *dev, bool state);

  /* Get phase currents */

  CODE int (*current_get)(FAR struct foc_dev_s *dev, FAR int16_t *curr_raw,
                          FAR foc_current_t *curr);

#ifdef CONFIG_MOTOR_FOC_TRACE
  /* FOC trace interface setup */

  CODE int (*trace_init)(FAR struct foc_dev_s *dev);

  /* FOC trace */

  CODE void (*trace)(FAR struct foc_dev_s *dev, int type, bool state);
#endif
};

/* Board-specific FOC data */

struct stm32_foc_board_data_s
{
  /* ADC configuration */

  FAR struct stm32_foc_adc_s *adc_cfg;

  /* PWM deadtime register value */

  uint8_t pwm_dt;

  /* PWM deadtime in ns */

  uint16_t pwm_dt_ns;

  /* PWM max supported duty cycle */

  foc_duty_t duty_max;
};

/* Board-specific FOC configuration */

struct stm32_foc_board_s
{
  /* Board-specific FOC operations */

  FAR struct stm32_foc_board_ops_s *ops;

  /* Board-specific FOC data */

  FAR struct stm32_foc_board_data_s *data;
};

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
 * Name: stm32_foc_initialize
 ****************************************************************************/

FAR struct foc_dev_s *
stm32_foc_initialize(int inst, FAR struct stm32_foc_board_s *board);

/****************************************************************************
 * Name: stm32_foc_adcget
 ****************************************************************************/

FAR struct adc_dev_s *stm32_foc_adcget(FAR struct foc_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_FOC_H */
