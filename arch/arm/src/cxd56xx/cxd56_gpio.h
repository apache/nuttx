/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gpio.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_GPIO_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cxd56_gpio_status_s
{
  bool input_en;
  bool output_en;
};

typedef struct cxd56_gpio_status_s cxd56_gpio_status_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
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
 * Name: cxd56_gpio_config
 *
 * Description:
 *   Configure a GPIO which input is enabled or not.
 *   Output is enabled when cxd56_gpio_write() is called.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpio_config(uint32_t pin, bool input_enable);

/****************************************************************************
 * Name: cxd56_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpio_write(uint32_t pin, bool value);

/****************************************************************************
 * Name: cxd56_gpio_write_hiz
 *
 * Description:
 *   Output HiZ to the selected opendrain GPIO pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpio_write_hiz(uint32_t pin);

/****************************************************************************
 * Name: cxd56_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Returned Value:
 *   The boolean state of the input pin
 *
 ****************************************************************************/

bool cxd56_gpio_read(uint32_t pin);

/****************************************************************************
 * Name: cxd56_gpio_status
 *
 * Description:
 *   Get a gpio status which input/output is enabled or not.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpio_status(uint32_t pin, cxd56_gpio_status_t *stat);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GPIO_H */
