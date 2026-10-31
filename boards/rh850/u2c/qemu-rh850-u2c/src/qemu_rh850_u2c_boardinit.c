/****************************************************************************
 * boards/rh850/u2c/qemu-rh850-u2c/src/qemu_rh850_u2c_boardinit.c
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

#include <stdint.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include "rh850_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QEMU rh850-u2c test-exit device (machine property test-exit=on) */

#define QEMU_TEST_EXIT_BASE  0x40000000
#define QEMU_TEST_EXIT_PASS  0x5555
#define QEMU_TEST_EXIT_FAIL  0x3333

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL_POWEROFF) || defined(CONFIG_QEMU_RH850_U2C_TEST_EXIT)
static void qemu_test_exit(int status, uint32_t zero)
{
  uint32_t value = zero;

  if (status != 0)
    {
      value = ((uint32_t)(status & 0xffff) << 16) | QEMU_TEST_EXIT_FAIL;
    }

  *(volatile uint32_t *)QEMU_TEST_EXIT_BASE = value;

  for (; ; )
    {
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   Board initialization after the OS is up: mount procfs.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#ifdef CONFIG_FS_PROCFS
  int ret;

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs: %d\n", ret);
    }
#endif
}
#endif

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Stop QEMU through the test-exit device: exit code 0 for status 0, else
 *   the status.  Without the device the write has no effect.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_POWEROFF
int board_power_off(int status)
{
  qemu_test_exit(status, QEMU_TEST_EXIT_PASS);
  return 0;
}
#endif

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Software system reset.  With QEMU_RH850_U2C_TEST_EXIT a non-zero status
 *   (the PANIC path) stops QEMU with that exit code instead.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET
int board_reset(int status)
{
#ifdef CONFIG_QEMU_RH850_U2C_TEST_EXIT
  if (status != 0)
    {
      qemu_test_exit(status, QEMU_TEST_EXIT_PASS);
    }
#endif

  up_systemreset();
  return 0;
}
#endif
