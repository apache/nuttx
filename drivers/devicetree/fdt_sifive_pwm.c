/****************************************************************************
 * drivers/devicetree/fdt_sifive_pwm.c
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

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>

#include <nuttx/fdt.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/timers/sifive_pwm.h>

#include <libfdt.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Read a node's single-cell "clock-frequency" (Hz).  The QEMU virt board
 * does not emit the property for this device (the model keeps the rate as a
 * QOM property), so fall back to the model default.  Parsed explicitly for
 * the same reason as in fdt_sp804.c: the generic helper loads the value
 * using the parent's #address-cells, which corrupts a 1-cell value.
 */

static uint32_t fdt_sifive_pwm_clock_hz(FAR const void *fdt, int offset)
{
  int len;
  FAR const fdt32_t *pv = fdt_getprop(fdt, offset, "clock-frequency", &len);
  uint32_t freq;

  if (pv == NULL || len < (int)sizeof(fdt32_t))
    {
      return SIFIVE_PWM_DEFAULT_FREQ;
    }

  freq = fdt32_ld(pv);
  return freq != 0 ? freq : SIFIVE_PWM_DEFAULT_FREQ;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_sifive_pwm_devices_register
 *
 * Description:
 *   Scan the device tree for "sifive,pwm0" nodes and register each as a PWM
 *   character device (/dev/pwm0, /dev/pwm1, ...).  The compare-match
 *   interrupts the node advertises are not used: the lower-half only
 *   programs the period and duty registers.
 *
 *   Every matching node is independent: the lower-half allocates its own
 *   state per call, so a board may expose any number of controllers (the
 *   arm virt board mounts one, sifive_u mounts two).  A node that cannot be
 *   brought up is reported and skipped rather than aborting the scan, so one
 *   malformed node does not cost the guest the controllers that follow it.
 *
 * Input Parameters:
 *   fdt - Device tree handle
 *
 * Returned Value:
 *   OK when every matching node was registered, or when the device tree
 *   carries no "sifive,pwm0" node at all.  A negated errno from the first
 *   node that failed otherwise.
 *
 ****************************************************************************/

int fdt_sifive_pwm_devices_register(FAR const void *fdt)
{
  FAR struct pwm_lowerhalf_s *lower;
  uintptr_t addr;
  uint32_t freq;
  char devpath[32];
  int offset = -1;
  int minor = 0;
  int err = OK;
  int ret;

  for (; ; )
    {
      offset = fdt_node_offset_by_compatible(fdt, offset, "sifive,pwm0");
      if (offset < 0)
        {
          /* -FDT_ERR_NOTFOUND ends the scan; anything else means the blob
           * itself is unusable and must not be fed back as an offset.
           */

          if (offset != -FDT_ERR_NOTFOUND && err == OK)
            {
              pwmerr("ERROR: fdt scan failed: %d\n", offset);
              err = -EINVAL;
            }

          break;
        }

      addr = fdt_get_reg_base(fdt, offset, 0);
      if (addr == 0)
        {
          pwmerr("ERROR: node at offset %d has no usable reg\n", offset);
          if (err == OK)
            {
              err = -EINVAL;
            }

          continue;
        }

      freq = fdt_sifive_pwm_clock_hz(fdt, offset);

      lower = sifive_pwm_initialize(addr, freq);
      if (lower == NULL)
        {
          pwmerr("ERROR: init failed for 0x%" PRIxPTR "\n", addr);
          if (err == OK)
            {
              err = -ENOMEM;
            }

          continue;
        }

      /* Claim the lowest free minor.  Minors are not derived from the node
       * address, so a skipped node does not leave a hole in the numbering.
       */

      snprintf(devpath, sizeof(devpath), "/dev/pwm%d", minor);
      ret = pwm_register(devpath, lower);
      if (ret < 0)
        {
          pwmerr("ERROR: %s register failed: %d\n", devpath, ret);
          sifive_pwm_uninitialize(lower);
          if (err == OK)
            {
              err = ret;
            }

          continue;
        }

      minor++;
    }

  return err;
}
