/****************************************************************************
 * arch/xtensa/src/esp32/esp32_region.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_REGION_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_REGION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_region_protection
 *
 * Description:
 *   Make page 0 access raise an exception.  Also protect some other unused
 *   pages so we can catch weirdness.
 *
 *   Useful attribute values:
 *     0  — cached, RW
 *     2  — bypass cache, RWX (default value after CPU reset)
 *     15 — no access, raise exception
 *
 ****************************************************************************/

void esp32_region_protection(void);

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_REGION_H */
