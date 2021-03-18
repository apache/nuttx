/****************************************************************************
 * boards/arm/lpc43xx/lpc4337-ws/src/lpc4337-ws.h
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

#ifndef __BOARDS_ARM_LPC43XX_LPC4337_WS_SRC_LPC4337_WS_H
#define __BOARDS_ARM_LPC43XX_LPC4337_WS_SRC_LPC4337_WS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "lpc43_pinconfig.h"
#include "lpc43_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#define HAVE_I2CTOOL 1
#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_ADC0
int lpc43_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC43XX_LPC4337_WS_SRC_LPC4337_WS_H */
