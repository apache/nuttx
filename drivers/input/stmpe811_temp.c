/****************************************************************************
 * drivers/input/stmpe811_temp.c
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

/* References:
 *   "STMPE811 S-Touch® advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/input/stmpe811.h>
#include <nuttx/random.h>

#include "stmpe811.h"

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE811) && !defined(CONFIG_STMPE811_TEMP_DISABLE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_tempinitialize
 *
 * Description:
 *  Configure the temperature sensor.
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe811_tempinitialize(STMPE811_HANDLE handle)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  uint8_t regval;

  /* Enable clocking for ADC and the temperature sensor */

  regval = stmpe811_getreg8(priv, STMPE811_SYS_CTRL2);
  regval &= ~(SYS_CTRL2_TS_OFF | SYS_CTRL2_ADC_OFF);
  stmpe811_putreg8(priv, STMPE811_SYS_CTRL2, regval);

  /* Enable the temperature sensor */

  stmpe811_putreg8(priv, STMPE811_TEMP_CTRL, TEMP_CTRL_ENABLE);

  /* Acquire data enable */

  stmpe811_putreg8(priv, STMPE811_TEMP_CTRL,
                   (TEMP_CTRL_ACQ | TEMP_CTRL_ENABLE));

  return OK;
}

/****************************************************************************
 * Name: stmpe811_tempread
 *
 * Description:
 *  Configure the temperature sensor.
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

uint16_t stmpe811_tempread(STMPE811_HANDLE handle)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  uint32_t temp = 0;
  uint8_t  temp1;
  uint8_t  temp2;

  /* Acquire data enable */

  stmpe811_putreg8(priv, STMPE811_TEMP_CTRL,
                   (TEMP_CTRL_ACQ | TEMP_CTRL_ENABLE));

  /* Read the temperature */

  temp1 = stmpe811_getreg8(priv, STMPE811_SYS_CTRL2);
  temp2 = stmpe811_getreg8(priv, STMPE811_SYS_CTRL2 + 1);

  add_sensor_randomness((temp1 << 8) | temp2);

  /* Scale the temperature (where Vio is assumed to be .33) */

  temp = ((uint32_t)(temp1 & 3) << 8) | temp2;
  temp = (uint32_t)((33 * temp * 100) / 751);
  temp = (uint32_t)((temp + 5) / 10);

  return (uint16_t)temp;
}

/****************************************************************************
 * Name: stmpe811_tempinterrupt
 *
 * Description:
 *  Configure the temperature sensor to sample the temperature periodically.
 *  Set the temperature threshold to generate an interrupt and notify
 *  to the client using the provide callback function pointer.
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   threshold - The threshold temperature value
 *   direction - True: Generate an interrupt if the temperate exceeds the
 *               threshold value; False:  Generate an interrupt if the
 *               temperature falls below the threshold value.
 *   callback  - The client callback function that will be called when
 *               the temperature crosses the threshold.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

/* Not implemented */

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE811 && !CONFIG_STMPE811_TEMP_DISABLE */
