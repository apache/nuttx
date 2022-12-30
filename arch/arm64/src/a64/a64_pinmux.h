/****************************************************************************
 * arch/arm64/src/a64/a64_pinmux.h
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

#ifndef __ARCH_ARM64_SRC_A64_A64_PINMUX_H
#define __ARCH_ARM64_SRC_A64_A64_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/a64_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2Cs *********************************************************************/

/* TWI0_SCK for PH0 */

#define PIO_TWI0_SCK (PIO_PERIPH2 | PIO_PULL_NONE | \
                      PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                      PIO_OUTPUT_CLEAR | PIO_PORT_PIOH | PIO_PIN0)

/* TWI0_SDA for PH1 */

#define PIO_TWI0_SDA (PIO_PERIPH2 | PIO_PULL_NONE | \
                      PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                      PIO_OUTPUT_CLEAR | PIO_PORT_PIOH | PIO_PIN1)

/* TWI1_SCK for PH2 */

#define PIO_TWI1_SCK (PIO_PERIPH2 | PIO_PULL_NONE | \
                      PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                      PIO_OUTPUT_CLEAR | PIO_PORT_PIOH | PIO_PIN2)

/* TWI1_SDA for PH3 */

#define PIO_TWI1_SDA (PIO_PERIPH2 | PIO_PULL_NONE | \
                      PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                      PIO_OUTPUT_CLEAR | PIO_PORT_PIOH | PIO_PIN3)

/* TWI2_SCK for PE14 */

#define PIO_TWI2_SCK (PIO_PERIPH3 | PIO_PULL_NONE | \
                      PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                      PIO_OUTPUT_CLEAR | PIO_PORT_PIOE | PIO_PIN14)

/* TWI2_SDA for PE15 */

#define PIO_TWI2_SDA (PIO_PERIPH3 | PIO_PULL_NONE | \
                      PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                      PIO_OUTPUT_CLEAR | PIO_PORT_PIOE | PIO_PIN15)

#endif /* __ARCH_ARM64_SRC_A64_A64_PINMUX_H */
