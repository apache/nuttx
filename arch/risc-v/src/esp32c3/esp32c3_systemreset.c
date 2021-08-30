/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_systemreset.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "hardware/esp32c3_rtccntl.h"
#include "esp32c3.h"
#include "esp32c3_systemreset.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SHUTDOWN_HANDLERS_NO 4

/****************************************************************************
 * Private Data
 ****************************************************************************/

static shutdown_handler_t shutdown_handlers[SHUTDOWN_HANDLERS_NO];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_register_shutdown_handler
 *
 * Description:
 *   This function allows you to register a handler that gets invoked before
 *   the application is restarted.
 *
 * Input Parameters:
 *   handler - Function to execute on restart
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp32c3_register_shutdown_handler(shutdown_handler_t handler)
{
  for (int i = 0; i < SHUTDOWN_HANDLERS_NO; i++)
    {
      if (shutdown_handlers[i] == handler)
        {
          return -EEXIST;
        }
      else if (shutdown_handlers[i] == NULL)
        {
          shutdown_handlers[i] = handler;
          return OK;
        }
    }

  return -ENOMEM;
}

/****************************************************************************
 * Name: esp32c3_unregister_shutdown_handler
 *
 * Description:
 *   This function allows you to unregister a handler which was previously
 *   registered using esp32c3_register_shutdown_handler function.
 *
 * Input Parameters:
 *   handler - Function to execute on restart
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp32c3_unregister_shutdown_handler(shutdown_handler_t handler)
{
  for (int i = 0; i < SHUTDOWN_HANDLERS_NO; i++)
    {
      if (shutdown_handlers[i] == handler)
        {
          shutdown_handlers[i] = NULL;
          return OK;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: up_shutdown_handler
 *
 * Description:
 *   Process all registered shutdown callback functions.
 *
 ****************************************************************************/

void up_shutdown_handler(void)
{
  for (int i = SHUTDOWN_HANDLERS_NO - 1; i >= 0; i--)
    {
      if (shutdown_handlers[i])
        {
          shutdown_handlers[i]();
        }
    }
}

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  putreg32(RTC_CNTL_SW_SYS_RST, RTC_CNTL_OPTIONS0_REG);

  /* Wait for the reset */

  for (; ; );
}
