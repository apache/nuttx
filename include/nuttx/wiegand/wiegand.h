/****************************************************************************
 * include/nuttx/wiegand/wiegand.h
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

#ifndef __INCLUDE_NUTTX_WIEGAND_WIEGAND_H
#define __INCLUDE_NUTTX_WIEGAND_WIEGAND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
struct wiegand_gpio_s
{
    uint32_t gpio;
    uint32_t id;
};

enum wiegand_status_e
{
  /* Parity sent was wrong. */

  WIEGAND_PARITY_ERROR = -2,

  /* The data read was uncompleted. */

  WIEGAND_READ_UNCOMPLETED = -1,

  /* The data read successfully. */

  WIEGAND_SUCCESS = 0
};

struct wiegand_data_s
{
    uint32_t id;
    int aba_code;
    uint32_t facility_code;
    enum wiegand_status_e status;
};

struct wiegand_config_s
{
    CODE int (*irq_attach)(FAR struct wiegand_config_s *dev, xcpt_t isr,
                            FAR void *arg);
    CODE void (*irq_enable)(FAR struct wiegand_config_s *dev, bool enable);
    CODE bool (*get_data)(FAR const struct wiegand_config_s *dev, int index);
    struct wiegand_gpio_s g_data[2];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: wiegand_register
 *
 * Description:
 *   Register the wiegand character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/wiega0"
 *   config  - The wiegand config.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int wiegand_register(FAR const char  *devpath,
                     FAR struct wiegand_config_s *config);
#endif /* __INCLUDE_NUTTX_WIEGAND_WIEGAND_H */