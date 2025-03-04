/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_efuse.c
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

#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <sys/param.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/efuse/efuse.h>

#include "chip.h"

#include "riscv_internal.h"
#include "espressif/esp_efuse.h"

#include "esp_efuse.h"
#include "esp_clk.h"
#include "hal/efuse_hal.h"
#include "esp_efuse_table.h"
#include "esp_efuse_chip.h"
#include "esp_efuse_utility.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_efuse_lowerhalf_s
{
  const struct efuse_ops_s *ops; /* Lower half operations */
  void *upper;                   /* Pointer to efuse_upperhalf_s */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* "Lower half" driver methods */

static int esp_efuse_lowerhalf_read(struct efuse_lowerhalf_s *lower,
                                    const efuse_desc_t *field[],
                                    uint8_t *data, size_t bits_len);
static int esp_efuse_lowerhalf_write(struct efuse_lowerhalf_s *lower,
                                     const efuse_desc_t *field[],
                                     const uint8_t *data,
                                     size_t bits_len);
static int esp_efuse_lowerhalf_ioctl(struct efuse_lowerhalf_s *lower,
                                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct efuse_ops_s g_esp_efuse_ops =
{
  .read_field   = esp_efuse_lowerhalf_read,
  .write_field  = esp_efuse_lowerhalf_write,
  .ioctl        = esp_efuse_lowerhalf_ioctl,
};

/* EFUSE lower-half */

static struct esp_efuse_lowerhalf_s g_esp_efuse_lowerhalf =
{
  .ops = &g_esp_efuse_ops,
  .upper = NULL,
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_efuse_lowerhalf_read
 *
 * Description:
 *   Read value from EFUSE, writing it into an array.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of
 *              the "lower-half" driver state structure
 *   field    - A pointer to describing the fields of efuse
 *   dst      - A pointer to array that contains the data for reading
 *   bits_len - The number of bits required to read
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int esp_efuse_lowerhalf_read(struct efuse_lowerhalf_s *lower,
                                    const efuse_desc_t *field[],
                                    uint8_t *data, size_t bits_len)
{
  int ret = OK;

  /* Read the requested field */

  ret = esp_efuse_read_field_blob((const esp_efuse_desc_t**)field,
                                  data,
                                  bits_len);

  return ret;
}

/****************************************************************************
 * Name: esp_efuse_lowerhalf_write
 *
 * Description:
 *   Write array to EFUSE.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of
 *              the "lower-half" driver state structure
 *   field    - A pointer to describing the fields of efuse
 *   data     - A pointer to array that contains the data for writing
 *   bits_len - The number of bits required to write
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int esp_efuse_lowerhalf_write(struct efuse_lowerhalf_s *lower,
                                     const efuse_desc_t *field[],
                                     const uint8_t *data,
                                     size_t bits_len)
{
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();

  ret = esp_efuse_write_field_blob((const esp_efuse_desc_t**)field,
                                   data,
                                   bits_len);

  leave_critical_section(flags);

  if (ret != OK)
    {
      return ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_efuse_lowerhalf_ioctl
 *
 * Description:
 *   Initialize the efuse driver. The efuse is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *                  the "lower-half" driver state structure
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int esp_efuse_lowerhalf_ioctl(struct efuse_lowerhalf_s *lower,
                                     int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* We don't have proprietary EFUSE ioctls */

      default:
        {
          minfo("Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_efuse_initialize
 *
 * Description:
 *   Initialize the efuse driver. The efuse is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the efuse device.
 *             This should be of the form /dev/efuse
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp_efuse_initialize(const char *devpath)
{
  struct esp_efuse_lowerhalf_s *lower = NULL;
  int ret = OK;

  DEBUGASSERT(devpath != NULL);

  lower = &g_esp_efuse_lowerhalf;

  /* Register the efuse upper driver */

  lower->upper = efuse_register(devpath,
                                (struct efuse_lowerhalf_s *)lower);

  if (lower->upper == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the efuse driver (such as if the
       * 'devpath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      ret = -EEXIST;
    }

#ifdef CONFIG_ESPRESSIF_EFUSE_VIRTUAL
  esp_efuse_utility_update_virt_blocks();
#endif

  return ret;
}
