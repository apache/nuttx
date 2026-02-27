/****************************************************************************
 * boards/avr/avrdx/breadxavr/src/avrdx_init.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#ifdef CONFIG_BREADXAVR_BUTTONS_DRIVER
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_BREADXAVR_TC74AX_SENSOR
#  include <nuttx/sensors/tc74ax.h>
#endif

#include <assert.h>
#include <avr/io.h>

#include "avrdx_gpio.h"
#include "avrdx_twi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_early_initialize
 *
 * Description:
 *  Function called by the OS when BOARD_EARLY_INITIALIZE is set.
 *  Called after up_initialize, OS has been initialized at this point
 *  and it is okay to initialize drivers. No waiting for events though.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_EARLY_INITIALIZE
void board_early_initialize(void)
{
#  ifdef CONFIG_BREADXAVR_BUTTONS_DRIVER
  int ret = OK;
#  endif

#  ifdef CONFIG_BREADXAVR_BUTTONS_DRIVER
  ret = btn_lower_initialize("/dev/buttons");
  if (ret != OK)
    {
      PANIC();
    }
#  endif
}

#endif /* CONFIG_BOARD_EARLY_INITIALIZE */

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *  Function called by the OS when BOARD_LATE_INITIALIZE is set.
 *  Called after up_initialize, OS has been initialized at this point
 *  and it is okay to initialize drivers. Running in special kernel
 *  thread so waiting is allowed. Executed just before application code.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#  ifdef CONFIG_BREADXAVR_TC74AX_SENSOR
  FAR struct i2c_master_s *i2c;
#  endif
  int ret = OK;

#  ifdef CONFIG_BREADXAVR_TC74AX_SENSOR

#    if  (CONFIG_BREADXAVR_TC74AX_SENSOR_VARIANT < 0) || \
         (CONFIG_BREADXAVR_TC74AX_SENSOR_VARIANT > 7)
#      error Incorrect setting of variant, Kconfig should not allow this
#    endif

  i2c = avrdx_initialize_twi(0);
  if (i2c)
    {
      ret = tc74ax_register("/dev/tc74ax",
                            i2c,
                            72 + CONFIG_BREADXAVR_TC74AX_SENSOR_VARIANT);
      if (ret != OK)
        {
          PANIC();
        }
    }
  else
    {
      PANIC();
    }

#  endif /* ifdef CONFIG_BREADXAVR_TC74AX_SENSOR */
}

#endif /* CONFIG_BOARD_LATE_INITIALIZE */
