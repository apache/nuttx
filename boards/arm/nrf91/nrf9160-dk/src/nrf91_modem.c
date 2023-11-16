/****************************************************************************
 * boards/arm/nrf91/nrf9160-dk/src/nrf91_modem.c
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

#include <debug.h>

#include "nrf_modem_at.h"
#include "nrf91_modem.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_modem_board_init
 ****************************************************************************/

int nrf91_modem_board_init(void)
{
  int ret;

  /* Configure COEX0 pin - on-board antena */

  ret = nrf_modem_at_printf("AT%%XCOEX0=1,1,1565,1586");
  if (ret < 0)
    {
      nerr("AT%%XCOEX0 config failed %d", ret);
      goto errout;
    }

  /* Configure MAGPIO pins */

  ret = nrf_modem_at_printf("AT%%XMAGPIO=1,0,0,1,1,1574,1577");
  if (ret < 0)
    {
      nerr("AT%%XMAGPIO config failed %d", ret);
      goto errout;
    }

errout:
  return ret;
}
