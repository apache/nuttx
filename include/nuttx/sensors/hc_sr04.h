/****************************************************************************
 * include/nuttx/sensors/hc_sr04.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_HC_SR04_H
#define __INCLUDE_NUTTX_SENSORS_HC_SR04_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Interrupt configuration data structure */

struct hcsr04_config_s
{
  CODE int  (*irq_attach)(FAR struct hcsr04_config_s * state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR const struct hcsr04_config_s *state,
                          bool enable);
  CODE void (*irq_clear)(FAR const struct hcsr04_config_s *state);
  CODE void (*irq_setmode)(FAR struct hcsr04_config_s *state, bool risemode);
  CODE void (*set_trigger)(FAR const struct hcsr04_config_s *state, bool on);
  CODE int64_t (*get_clock)(FAR const struct hcsr04_config_s *state);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int hcsr04_register(FAR const char *devpath,
                    FAR struct hcsr04_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_HC_SR04_H */
