/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_ulp.c
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
#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "ulp_lp_core.h"
#include "ulp/ulp_var_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_ulp_ioctl(struct file *filep, int cmd, unsigned long arg);
static int esp_ulp_write(struct file *filep,
                         const char *buffer,
                         size_t buflen);
int esp_ulp_load_bin(const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_esp_ulp_fops =
{
  .write = esp_ulp_write, /* write */
  .ioctl = esp_ulp_ioctl, /* ioctl */
};

/* Configuration for ULP LP Core */

ulp_lp_core_cfg_t cfg =
{
  .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_ulp_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   filep - The pointer of file, represents each user using the sensor
 *   cmd   - The ioctl command
 *   arg   - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_ulp_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);
  int ret = 0;
  int index = -1;
  struct symtab_s *sym = (struct symtab_s *)arg;
  int var_map_size = sizeof(ulp_var_map) / sizeof(ulp_var_map[0]);

  DEBUGASSERT(sym);

  /* Decode and dispatch the driver-specific IOCTL command */

  for (int i = 0; i < var_map_size; i++)
    {
      if (strcmp(ulp_var_map[i].sym.sym_name, sym->sym_name) == 0)
        {
          index = i;
          break;
        }
    }

  if (index == -1)
    {
      ferr("Symbol name does not exist\n");
      return ERROR;
    }

  switch (cmd)
    {
      case FIONREAD:
        memcpy((void *)sym->sym_value, ulp_var_map[index].sym.sym_value,
               ulp_var_map[index].size);
        break;

      case FIONWRITE:
        memcpy((void *)ulp_var_map[index].sym.sym_value, sym->sym_value,
               ulp_var_map[index].size);
        break;

      default:
        ferr("Unrecognized IOCTL command: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_ulp_write
 *
 * Description:
 *   Load binary data into ULP.
 *
 * Input Parameters:
 *   filep  - The pointer of file
 *   buffer - Buffer that includes binary to run on ULP.
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_ulp_write(struct file *filep,
                         const char *buffer,
                         size_t buflen)
{
  UNUSED(filep);
  return esp_ulp_load_bin(buffer, buflen);
}

/****************************************************************************
 * Name: esp_ulp_register
 *
 * Description:
 *   This function registers ULP.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_ulp_register(void)
{
  register_driver("/dev/ulp", &g_esp_ulp_fops, 0666, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_ulp_load_bin
 *
 * Description:
 *   Load binary data into ULP.
 *
 * Input Parameters:
 *   buffer - Buffer that includes binary to run on ULP.
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int esp_ulp_load_bin(const char *buffer, size_t buflen)
{
  int ret = ERROR;
  ulp_lp_core_stop();
  ret = ulp_lp_core_load_binary((const uint8_t *)buffer, buflen);
  ulp_lp_core_run(&cfg);
  return ret;
}

/****************************************************************************
 * Name: esp_ulp_init
 *
 * Description:
 *   Initialize ULP co-processor
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_ulp_init(void)
{
  esp_ulp_register();
  ulp_lp_core_run(&cfg);
}
