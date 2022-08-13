/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_pmic.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#include "sam_twi.h"

#include "sama5d4-ek.h"

#ifdef HAVE_PMIC

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pmic_initialize
 *
 * Description:
 *   Currently, this function only disables the PMIC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_pmic_initialize(void)
{
  struct i2c_master_s *i2c;
  struct i2c_config_s config;
  uint8_t buffer[2];

  /* Get an instance of the I2C interface for the PMIC */

  i2c = sam_i2cbus_initialize(PMIC_TWI_BUS);
  if (!i2c)
    {
      _err("ERROR: Failed to initialize TWI%d\n", PMIC_TWI_BUS);
    }
  else
    {
      /* Setup up the I2C configuration */

      config.frequency = PMIC_I2C_FREQUENCY;
      config.address   = PMIC_I2C_ADDRESS;
      config.addrlen   = 7;

      /* Send the disable sequence */

      buffer[0] = 0x0b;
      buffer[1] = 0xee;
      i2c_write(i2c, &config, buffer, 2);

      buffer[0] = 0x02;
      buffer[1] = 0x0f;
      i2c_write(i2c, &config, buffer, 2);

      buffer[0] = 0x03;
      buffer[1] = 0x0f;
      i2c_write(i2c, &config, buffer, 2);
    }
}

#endif /* HAVE_PMIC */
