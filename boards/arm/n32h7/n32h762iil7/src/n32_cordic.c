/****************************************************************************
 * boards/arm/n32h7/n32h762iil7/src/n32_cordic.c
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

#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/math/cordic.h>
#include <arch/board/board.h>

#include "n32_cordic.h"

#include "n32h762iil7.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_cordic_setup
 *
 * Description:
 *   Initialize CORDIC and register the CORDIC device.
 *
 ****************************************************************************/

int n32_cordic_setup(void)
{
  struct cordic_lowerhalf_s *cordic      = NULL;
  static bool                initialized = false;
  int                        ret         = OK;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call n32_cordicinitialize() to get an instance of the CORDIC
       * interface
       */

      cordic = n32_cordicinitialize();
      if (!cordic)
        {
          tmrerr("Failed to get the N32 CORDIC lower half\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Register the CORDIC driver at "/dev/cordic0" */

      ret = cordic_register("/dev/cordic0", cordic);
      if (ret < 0)
        {
          tmrerr("cordic_register failed: %d\n", ret);
          goto errout;
        }

      /* Now we are initialized */

      initialized = true;
    }

errout:
  return ret;
}
