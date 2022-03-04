/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_wdt_lowerhalf.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WDT_LOWERHALF_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WDT_LOWERHALF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s3_wdt.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_wdt_initialize
 *
 * Description:
 *   Initialize the watchdog timer.  The watchdog timer is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.
 *   wdt     - WDT instance to be initialized.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp32s3_wdt_initialize(const char *devpath, enum esp32s3_wdt_inst_e wdt);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WDT_LOWERHALF_H */
