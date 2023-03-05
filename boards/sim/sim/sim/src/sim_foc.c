/****************************************************************************
 * boards/sim/sim/sim/src/sim_foc.c
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

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/motor/foc/foc_dummy.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_foc_setup
 *
 * Description:
 *   Initialize the FOC device.
 *
 *   This function should be call by board_app_initialize().
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int sim_foc_setup(void)
{
  struct foc_dev_s *foc[CONFIG_MOTOR_FOC_INST];
  static bool           initialized = false;
  int                   ret         = OK;
  int                   i           = 0;
  char                  devpath[20];

  mtrinfo("sim_foc_setup\n");

  /* Initialize only once */

  if (!initialized)
    {
      /* Register devices */

      for (i = 0; i < CONFIG_MOTOR_FOC_INST; i += 1)
        {
          /* Initialize arch specific FOC lower-half */

          foc[i] = foc_dummy_initialize(i);
          if (foc[i] == NULL)
            {
              ret = -errno;
              mtrerr("Failed to initialize sim FOC%d: %d\n", i, ret);
              goto errout;
            }

          DEBUGASSERT(foc[i]->lower);

          /* Get devpath for FOC */

          snprintf(devpath, sizeof(devpath), "/dev/foc%d", i);

          /* Register FOC device */

          ret = foc_register(devpath, foc[i]);
          if (ret < 0)
            {
              mtrerr("Failed to register FOC device %s: %d\n",
                     devpath, ret);
              goto errout;
            }
        }

      initialized = true;
    }

errout:
  return ret;
}
