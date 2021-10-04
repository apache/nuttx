/****************************************************************************
 * arch/xtensa/src/esp32/esp32_efuse_lowerhalf.c
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
#include <assert.h>
#include <nuttx/kmalloc.h>
#include <nuttx/efuse/efuse.h>

#include "hardware/esp32_soc.h"
#include "esp32_efuse.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

struct esp32_efuse_lowerhalf_s
{
  const struct efuse_ops_s *ops; /* Lower half operations */
  void *upper;                   /* Pointer to efuse_upperhalf_s */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int      esp32_efuse_read_field(struct efuse_lowerhalf_s *lower,
                                       const efuse_desc_t *field[],
                                       uint8_t *data, size_t size);
static int      esp32_efuse_write_field(struct efuse_lowerhalf_s *lower,
                                        const efuse_desc_t *field[],
                                        const uint8_t *data,
                                        size_t size);
static int      efuse_ioctl(struct efuse_lowerhalf_s *lower, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct efuse_ops_s g_esp32_efuse_ops =
{
  .read_field   = esp32_efuse_read_field,
  .write_field  = esp32_efuse_write_field,
  .ioctl        = efuse_ioctl,
};

/* EFUSE lower-half */

static struct esp32_efuse_lowerhalf_s g_esp32_efuse_lowerhalf =
{
  .ops = &g_esp32_efuse_ops,
  .upper = NULL,
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

static int esp32_efuse_read_field(struct efuse_lowerhalf_s *lower,
                                  const efuse_desc_t *field[],
                                  uint8_t *data, size_t bits_len)
{
  int ret = OK;

  /* Read the requested field */

  ret = esp_efuse_read_field(field, data, bits_len);

  return ret;
}

static int esp32_efuse_write_field(struct efuse_lowerhalf_s *lower,
                                   const efuse_desc_t *field[],
                                   const uint8_t *data, size_t bits_len)
{
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();

  /* Write the blob data to the field */

  ret = esp_efuse_write_field(field, data, bits_len);

  /* Burn the EFUSEs */

  esp_efuse_burn_efuses();

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: efuse_ioctl
 ****************************************************************************/

static int efuse_ioctl(struct efuse_lowerhalf_s *lower,
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
 * Name: esp32_efuse_initialize
 *
 * Description:
 *   Initialize the efuse driver.  The efuse is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   devpath                 - The full path to the efuse.  This should
 *                             be of the form /dev/efuse
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp32_efuse_initialize(const char *devpath)
{
  struct esp32_efuse_lowerhalf_s *lower = NULL;
  int ret = OK;

  DEBUGASSERT(devpath);

  lower = &g_esp32_efuse_lowerhalf;

  /* Register the efuser upper driver */

  lower->upper = efuse_register(devpath,
                                (struct efuse_lowerhalf_s *)lower);

  if (lower->upper == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the efuser driver (such as if the
       * 'devpath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      ret = -EEXIST;
      goto errout;
    }

errout:
  return ret;
}
