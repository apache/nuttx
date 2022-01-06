/****************************************************************************
 * arch/xtensa/include/esp32s2/chip.h
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

#ifndef __ARCH_XTENSA_INCLUDE_ESP32S2_CHIP_H
#define __ARCH_XTENSA_INCLUDE_ESP32S2_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Characterize each supported ESP32S2 part */

#define ESP32S2_NDAC     2  /* DAC0-1 */
#define ESP32S2_NI2C     1  /* I2C0 */
#define ESP32S2_NI2S     1  /* I2S0 */
#define ESP32S2_NLCD     1  /* LCD0 */
#define ESP32S2_NSPI     4  /* SPI0-3 */
#define ESP32S2_NUARTS   2  /* UART0-1 */
#define ESP32S2_NUSBOTG  1  /* USB OTG */

#define ESP32S2_NGPIOS   46 /* GPIO0-45 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_INCLUDE_ESP32S2_CHIP_H */
