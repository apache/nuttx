/****************************************************************************
 * arch/arm64/src/am62x/hardware/am62x_gpio.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_GPIO_H
#define __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/am62x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM62X_GPIO_NBANKS                  2
#define AM62X_GPIO_NPINS                   32

#define AM62X_GPIO_BINTEN_OFFSET           0x0008

/* AM62x main GPIO uses the Davinci/Keystone register layout.  The Linux
 * binding for k3-am62-main.dtsi advertises compatible = "ti,am64-gpio",
 * "ti,keystone-gpio" and a 0x100-byte register window.
 */

#define AM62X_GPIO_BANK_STRIDE             0x0028
#define AM62X_GPIO_BANK_OFFSET(n)          (0x0010 + \
                                            (n) * AM62X_GPIO_BANK_STRIDE)
#define AM62X_GPIO_DIR_OFFSET(n)           (AM62X_GPIO_BANK_OFFSET(n) + 0x00)
#define AM62X_GPIO_OUT_DATA_OFFSET(n)      (AM62X_GPIO_BANK_OFFSET(n) + 0x04)
#define AM62X_GPIO_SET_DATA_OFFSET(n)      (AM62X_GPIO_BANK_OFFSET(n) + 0x08)
#define AM62X_GPIO_CLR_DATA_OFFSET(n)      (AM62X_GPIO_BANK_OFFSET(n) + 0x0c)
#define AM62X_GPIO_IN_DATA_OFFSET(n)       (AM62X_GPIO_BANK_OFFSET(n) + 0x10)
#define AM62X_GPIO_SET_RIS_TRIG_OFFSET(n)  \
                                            (AM62X_GPIO_BANK_OFFSET(n) + 0x14)
#define AM62X_GPIO_CLR_RIS_TRIG_OFFSET(n)  \
                                            (AM62X_GPIO_BANK_OFFSET(n) + 0x18)
#define AM62X_GPIO_SET_FAL_TRIG_OFFSET(n)  \
                                            (AM62X_GPIO_BANK_OFFSET(n) + 0x1c)
#define AM62X_GPIO_CLR_FAL_TRIG_OFFSET(n)  \
                                            (AM62X_GPIO_BANK_OFFSET(n) + 0x20)
#define AM62X_GPIO_INTSTAT_OFFSET(n)       (AM62X_GPIO_BANK_OFFSET(n) + 0x24)

#define AM62X_GPIO_BINTEN(b)               ((b) + AM62X_GPIO_BINTEN_OFFSET)
#define AM62X_GPIO_DIR(b, n)               ((b) + AM62X_GPIO_DIR_OFFSET(n))
#define AM62X_GPIO_OUT_DATA(b, n)          ((b) + AM62X_GPIO_OUT_DATA_OFFSET(n))
#define AM62X_GPIO_SET_DATA(b, n)          ((b) + AM62X_GPIO_SET_DATA_OFFSET(n))
#define AM62X_GPIO_CLR_DATA(b, n)          ((b) + AM62X_GPIO_CLR_DATA_OFFSET(n))
#define AM62X_GPIO_IN_DATA(b, n)           ((b) + AM62X_GPIO_IN_DATA_OFFSET(n))
#define AM62X_GPIO_SET_RIS_TRIG(b, n)      ((b) + AM62X_GPIO_SET_RIS_TRIG_OFFSET(n))
#define AM62X_GPIO_CLR_RIS_TRIG(b, n)      ((b) + AM62X_GPIO_CLR_RIS_TRIG_OFFSET(n))
#define AM62X_GPIO_SET_FAL_TRIG(b, n)      ((b) + AM62X_GPIO_SET_FAL_TRIG_OFFSET(n))
#define AM62X_GPIO_CLR_FAL_TRIG(b, n)      ((b) + AM62X_GPIO_CLR_FAL_TRIG_OFFSET(n))
#define AM62X_GPIO_INTSTAT(b, n)           ((b) + AM62X_GPIO_INTSTAT_OFFSET(n))

#define AM62X_GPIO_BIT(n)                  (1u << ((n) & 31))

#endif /* __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_GPIO_H */
