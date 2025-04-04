/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_dedic_gpio.h
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
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_DEDIC_GPIO_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_DEDIC_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct esp_dedic_gpio_flags_s
{
  bool input_enable;
  bool invert_input_enable;
  bool output_enable;
  bool invert_output_enable;
};

struct esp_dedic_gpio_config_s
{
  const int *gpio_array;                /* Array of GPIO numbers */
  int array_size;                       /* Number of GPIOs in gpio_array */
  struct esp_dedic_gpio_flags_s *flags; /* Flags to control specific behaviour of GPIO bundle */
  char *path;                           /* Path for character driver */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_dedic_gpio_write
 *
 * Description:
 *   Write data to the dedicated gpio bundle with given mask value.
 *
 * Input Parameters:
 *   dev    - Pointer to the dedicated gpio driver struct
 *   mask   - Mask of the GPIOs to be written
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_dedic_gpio_write(struct file *dev,
                          uint32_t mask,
                          uint32_t value);

/****************************************************************************
 * Name: esp_dedic_gpio_read
 *
 * Description:
 *   Read dedicated gpio bundle data.
 *
 * Input Parameters:
 *   dev    - Pointer to the dedicated gpio driver struct
 *   value  - Pointer to the read data will be saved
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_dedic_gpio_read(struct file *dev, int *value);

/****************************************************************************
 * Name: esp_dedic_gpio_new_bundle
 *
 * Description:
 *   Request dedicated GPIO bundle and config it with given parameters.
 *
 * Input Parameters:
 *   config - Dedicated GPIO bundle configuration
 *
 * Returned Value:
 *   Valid GPIO device structure reference on success; NULL on failure.
 *
 ****************************************************************************/

struct file *esp_dedic_gpio_new_bundle(
    struct esp_dedic_gpio_config_s *config);

/****************************************************************************
 * Name: esp_dedic_gpio_del_bundle
 *
 * Description:
 *   Delete dedicated gpio bundle.
 *
 * Input Parameters:
 *   dev - Pointer to the dedicated gpio driver struct
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_dedic_gpio_del_bundle(struct file *dev);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_DEDIC_GPIO_H */
