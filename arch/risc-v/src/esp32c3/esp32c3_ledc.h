/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_ledc.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_LEDC_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_LEDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>

/****************************************************************************
 * Public functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_ledc_init
 *
 * Description:
 *   Initialize one LEDC timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.
 *
 * Returned Value:
 *   On success, a pointer to the ESP32-C3 LEDC lower half PWM driver is
 *   returned. NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *esp32c3_ledc_init(int timer);

#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_LEDC_H */
