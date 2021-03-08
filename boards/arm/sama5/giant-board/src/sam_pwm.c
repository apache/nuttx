/****************************************************************************
 * boards/arm/sama5/giant-board/src/sam_pwm.c
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
#include "giant-board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM.
 * There are no dedicated PWM output pins available to the user for PWM
 * testing.
 * Care must be taken because all PWM output pins conflict with some other
 * usage of the pin by other devices:
 *
 *    -----+---+---+----+--------------------
 *     PWM  PIN PER PIO  CONFLICTS
 *    -----+---+---+----+--------------------
 *     PWM0 FI   B  PC28   SPI1, ISI
 *          H    B  PB0    GMAC
 *               B  PA20   LCDC, ISI
 *          L    B  PB1    GMAC
 *               B  PA21   LCDC, ISI
 *    -----+---+---+----+--------------------
 *     PWM1 FI   B  PC31   HDMI
 *          H    B  PB4    GMAC
 *               B  PA22   LCDC, ISI
 *          L    B  PB5    GMAC
 *               B  PE31   ISI, HDMI
 *               B  PA23   LCDC, ISI
 *    -----+---+---+----+--------------------
 *     PWM2 FI   B  PC29   UART0, ISI, HDMI
 *          H    C  PD5    HSMCI0
 *               B  PB8    GMAC
 *          L    C  PD6    HSMCI0
 *               B  PB9    GMAC
 *    -----+---+---+----+--------------------
 *     PWM3 FI   C  PD16   SPI0, Audio
 *          H    C  PD7    HSMCI0
 *               B  PB12   GMAC
 *          L    C  PD8    HSMCI0
 *               B  PB13   GMAC
 *    -----+---+---+----+--------------------
 */

#ifndef CONFIG_GIANT_BOARD_CHANNEL
#  if defined(CONFIG_SAMA5_PWM_CHAN0)
#    warning Assuming PWM channel 0
#    define CONFIG_GIANT_BOARD_CHANNEL 0
#  elif defined(CONFIG_SAMA5_PWM_CHAN1)
#    warning Assuming PWM channel 1
#    define CONFIG_GIANT_BOARD_CHANNEL 1
#  elif defined(CONFIG_SAMA5_PWM_CHAN2)
#    warning Assuming PWM channel 2
#    define CONFIG_GIANT_BOARD_CHANNEL 2
#  elif defined(CONFIG_SAMA5_PWM_CHAN3)
#    warning Assuming PWM channel 3
#    define CONFIG_GIANT_BOARD_CHANNEL 3
#  endif
#endif

#if defined(CONFIG_PWM) && defined(CONFIG_SAMA5_PWM)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int sam_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call sam_pwminitialize() to get an instance of the PWM interface */

      pwm = sam_pwminitialize(CONFIG_SAMA5D3XPLAINED_CHANNEL);
      if (!pwm)
        {
          _err("ERROR: Failed to get the SAMA5 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
