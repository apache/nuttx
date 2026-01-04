/****************************************************************************
 * arch/sim/src/sim/sim_gpiochip.h
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

#ifndef __ARCH_SIM_SRC_SIM_GPIOCHIP_H
#define __ARCH_SIM_SRC_SIM_GPIOCHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef __SIM__
#  include "config.h"
#endif

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIOCHIP_LINE_FLAG_DISABLE  (1 << 0)
#define GPIOCHIP_LINE_FLAG_RISING   (1 << 1)
#define GPIOCHIP_LINE_FLAG_FALLING  (1 << 2)
#define GPIOCHIP_LINE_FLAG_BOTH     (1 << 3)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct host_gpiochip_dev
{
  int file;
  int line_fd[CONFIG_IOEXPANDER_NPINS];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct host_gpiochip_dev *host_gpiochip_alloc(const char *filename);
void host_gpiochip_free(struct host_gpiochip_dev *dev);
int host_gpiochip_get_line(struct host_gpiochip_dev *priv, uint8_t pin,
                           bool *input);
int host_gpiochip_readpin(struct host_gpiochip_dev *dev,
                          uint8_t line, bool *value);
int host_gpiochip_writepin(struct host_gpiochip_dev *dev,
                           uint8_t pin, bool value);
int host_gpiochip_direction(struct host_gpiochip_dev *dev,
                            uint8_t pin, bool input);
int host_gpiochip_irq_request(struct host_gpiochip_dev *dev, uint8_t pin,
                              uint16_t cfgset);
bool host_gpiochip_irq_active(struct host_gpiochip_dev *dev, uint8_t pin);

#endif /* __ARCH_SIM_SRC_SIM_GPIOCHIP_H */
