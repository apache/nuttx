/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_bringup.c
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
#include <sys/types.h>
#include <syslog.h>
#ifdef CONFIG_I2C
#  include <nuttx/i2c/i2c_master.h>
#endif
#include <nuttx/kmalloc.h>
#ifdef CONFIG_MPU60X0_I2C
#  include <nuttx/sensors/mpu60x0.h>
#endif

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#  include "pinephone_display.h"
#endif

#include "a64_twi.h"
#include "pinephone.h"
#include "pinephone_pmic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int pinephone_bringup(void)
{
  int ret;
#ifdef CONFIG_I2C
  int i2c_bus;
  struct i2c_master_s *i2c;
#ifdef CONFIG_MPU60X0_I2C
  struct mpu_config_s *mpu_config;
#endif
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }

  /* Render the Test Pattern */

  pinephone_display_test_pattern();
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI1)
  i2c_bus = 1;
  i2c = a64_i2cbus_initialize(i2c_bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", i2c_bus);
    }
  else
    {
#if defined(CONFIG_SYSTEM_I2CTOOL)
      ret = i2c_register(i2c, i2c_bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 i2c_bus, ret);
        }
#endif

#ifdef CONFIG_MPU60X0_I2C
      /* Init PMIC */

      ret = pinephone_pmic_init();
      if (ret < 0)
        {
          syslog(LOG_ERR, "Init PMIC failed: %d\n", ret);
          return ret;
        }

      /* Wait 15 milliseconds for power supply and power-on init */

      up_mdelay(15);

      mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
      if (mpu_config == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to allocate mpu60x0 driver\n");
        }
      else
        {
          mpu_config->i2c = i2c;
          mpu_config->addr = 0x68;
          mpu60x0_register("/dev/imu0", mpu_config);
        }
#endif
    }
#endif

  UNUSED(ret);
  return OK;
}
