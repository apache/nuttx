/****************************************************************************
 * arch/risc-v/src/k210/k210_gpiohs.h
 *
 * Derives from software originally provided by Canaan Inc
 *
 *   Copyright 2018 Canaan Inc
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

#ifndef __ARCH_RISCV_SRC_K210_K210_GPIOHS_H
#define __ARCH_RISCV_SRC_K210_K210_GPIOHS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: k210_gpiohs_set_direction
 *
 * Description:
 *   Set gpiohs direction
 *
 * Input Parameters:
 *   io  - IO number
 *   dir - true for output, false for input
 *
 ****************************************************************************/

void k210_gpiohs_set_direction(uint32_t io, bool dir);

/****************************************************************************
 * Name: k210_gpiohs_set_value
 *
 * Description:
 *   Set gpiohs direction
 *
 * Input Parameters:
 *   io  - IO number
 *   dir - true for high level, false for low level
 *
 ****************************************************************************/

void k210_gpiohs_set_value(uint32_t io, bool val);

/****************************************************************************
 * Name: k210_gpiohs_get_value
 *
 * Description:
 *   Get gpiohs level
 *
 * Input Parameters:
 *   io  - IO number
 *
 * Returned Value:
 *   true for high level, false for low level
 *
 ****************************************************************************/

bool k210_gpiohs_get_value(uint32_t io);

#endif /* __ARCH_RISCV_SRC_K210_K210_GPIOHS_H */
