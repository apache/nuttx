/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_gpio.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include <nuttx/wireless/ieee80211/bcmf_gpio.h>

#include "bcmf_cdc.h"
#include "bcmf_interface.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_set_gpio
 ****************************************************************************/

int bcmf_set_gpio(FAR struct bcmf_dev_s *priv, int pin, bool value)
{
  struct
    {
      uint32_t mask;
      uint32_t value;
    } buffer;

  uint32_t buf_len = sizeof(buffer);

  if (!(((FAR bcmf_interface_dev_t *)priv->bus)->ready)) return -EIO;

  buffer.mask  = 1 << pin;
  buffer.value = value ? (1 << pin) : 0;

  return bcmf_cdc_iovar_request(priv,
                                CHIP_STA_INTERFACE,
                                true,
                                IOVAR_STR_GPIOOUT,
                                (uint8_t *)&buffer,
                                &buf_len);
}

/****************************************************************************
 * Name: bcmf_get_gpio
 ****************************************************************************/

int bcmf_get_gpio(FAR struct bcmf_dev_s *priv, int pin, bool *value)
{
  uint8_t  buffer;
  uint32_t buf_len = sizeof(buffer);
  int      ret;

  if (!(((FAR bcmf_interface_dev_t *)priv->bus)->ready)) return -EIO;

  ret = bcmf_cdc_iovar_request(priv,
                               CHIP_STA_INTERFACE,
                               false,
                               IOVAR_STR_CCGPIOIN,
                               (uint8_t *)&buffer,
                               &buf_len);
  if (ret != OK) return ret;

  *value = (buffer & (1 << pin)) != 0;

  return OK;
}
