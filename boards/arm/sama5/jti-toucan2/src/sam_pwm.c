/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/sam_pwm.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "sam_pwm.h"
#include "jti-toucan2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM.
 * There are dedicated PWM output pins available to the user for PWM
 * Care must be taken because the PWM output pins conflict with some other
 * usage of the pin by other devices:
 *
 */

#if defined(CONFIG_PWM)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device(s).
 *
 ****************************************************************************/
static bool initialized[4] = {false, false, false, false};
int sam_pwm_setup(int pwm_channel)
{
  
  struct pwm_lowerhalf_s *pwm;
  int ret;

  if (pwm_channel >= 4)
	{
	  _err("ERROR: requested PWM channel out of range\n");
	  ret = EINVAL;
	}
	  
  /* Have we already initialized? */

  if (!initialized[pwm_channel])
    {
      /* Call sam_pwminitialize() to get an instance of the PWM interface */

      pwm = sam_pwminitialize(pwm_channel);
      if (!pwm)
        {
          _err("ERROR: Failed to get the SAMA5 PWM lower half for channel %d\n", pwm_channel);
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      switch (pwm_channel)
	    {
		  case 0:
			ret = pwm_register("/dev/pwm0", pwm);
		  break;
		  case 1:
			ret = pwm_register("/dev/pwm1", pwm);
		  break;
		  case 2:
			ret = pwm_register("/dev/pwm2", pwm);
		  break;
		  case 3:
			ret = pwm_register("/dev/pwm3", pwm);
		  break;
		  default:
		    ret = EINVAL;
      break;
		}
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed for channel %d: %d\n", pwm_channel, ret);
          return ret;
        }

      /* Now we are initialized */

      initialized[pwm_channel] = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
