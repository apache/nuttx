/****************************************************************************
 * arch/xtensa/src/esp32/esp32_dac.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_DAC_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/dac.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_dac_initialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Valid dac device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

struct dac_dev_s *esp32_dac_initialize(void);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_DAC_H */
