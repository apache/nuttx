/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_analog.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_ANALOG_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_ANALOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_analog_read
 ****************************************************************************/

uint8_t locate_code(".ram_code") tlsr82_analog_read(uint8_t addr);

/****************************************************************************
 * Name: tlsr82_analog_write
 ****************************************************************************/

void locate_code(".ram_code") tlsr82_analog_write(uint8_t addr, uint8_t val);

/****************************************************************************
 * Name: tlsr82_analog_modify
 ****************************************************************************/

void locate_code(".ram_code") tlsr82_analog_modify(uint8_t addr,
                                                   uint8_t mask,
                                                   uint8_t val);

#endif /* __ARCH_ARM_SRC_TLSR82_TLSR82_ANALOG_H */
