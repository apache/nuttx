/****************************************************************************
 * arch/arm/src/imxrt/imxrt_ocotp.c
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
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/clock.h>

#include <arch/board/board.h>
#include "arm_internal.h"
#include "imxrt_periphclks.h"
#include "imxrt_ocotp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OCOTP Timing */

#define OCOTP_OPT_TIMEOUT_MS                  100

/****************************************************************************
 * Private Function
 ****************************************************************************/

static inline void imxrt_ocotp_reset_errors(void)
{
  putreg32(OCOTP_CTRL_ERROR, IMXRT_OCOTP_CTRL_CLR);
}

static void imxrt_ocotp_initialize(void)
{
  static bool once = false;
  uint32_t read;
  uint32_t prog;
  uint32_t relax;
  uint32_t relax_read;
  uint32_t relax_prog;
  uint32_t wait;

  const uint32_t ipg_freq_hz = BOARD_CPU_FREQUENCY / IMXRT_IPG_PODF_DIVIDER;

  if (!once)
    {
      once = true;

      imxrt_clockall_ocotp_ctrl();

      /* WAIT specifies time interval between auto read and write
       *      access in one time program.
       *
       * Subject to:
       *   tSP_RD = (WAIT+1)/ipg_clk_freq, should be ≧ 150 ns.
       *     WAIT = (150ns * ipg_clk_freq) -1
       */

      wait = (((uint64_t)(150LL * ipg_freq_hz) + 1000000000LL)
              / 1000000000LL) - 1;

      /* RELAX is used to configure the setup/hold time for certain
       *       timing margin.
       *
       * Subject to:
       *   tSP_PGM = tHP_PGM = (RELAX+1)/ipg_clk_freq, should be ≧ 100 ns.
       *     RELAX = (100ns * ipg_clk_freq) -1
       */

      relax = (((uint64_t)(100LL * ipg_freq_hz) + 1000000000LL)
               / 1000000000LL) - 1;

      /* RELAX_READ is used to configure the setup/hold time for
       *            certain timing margin.
       *
       * Subject to:
       *     (RELAX_READ+1)/ipg_clk_freq should be ≧ 10 ns.
       *     RELAX_READ = (10ns * ipg_clk_freq) -1
       */

      relax_read = (((uint64_t)(10LL * ipg_freq_hz) + 1000000000LL)
                     / 1000000000LL) - 1;

      /* RELAX_PROG is used to configure the setup/hold time for
       *            certain timing margin.
       *
       * Subject to:
       *     tSP_PG_AVDD = tHP_PG_AVDD = (RELAX_PROG+1)/ipg_clk_freq,
       *     should be ≧ 1000 ns.
       *
       *     RELAX_PROG = (1000 ns * ipg_clk_freq) -1
       */

      relax_prog = (((uint64_t)(1000LL * ipg_freq_hz) + 1000000000LL)
                    / 1000000000LL) - 1;

      /* STROBE_PROG configure the program strobe
       *
       * Subject to:
       *
       *  The tPGM should be configured within the range of 9000 ns <
       *  tPGM < 11000 ns, while its recommended value is 10000 ns.
       *
       *  tPGM  = [(STROBE_PROG+1) – 2×(RELAX_PROG+1)]/ipg_clk_freq.
       *  10000 =  [(STROBE_PROG+1) – 2×(RELAX_PROG+1)]/ipg_clk_freq.
       *  STROBE_PROG =  (10000 * ipg_clk_freq) + 2×(RELAX_PROG+1)) - 1
       *
       */

      prog = (((uint64_t)(10000LL * ipg_freq_hz) + 1000000000LL) /
             1000000000LL + 2 * (relax_prog + 1)) - 1;

      /* STROBE_READ configure the program strobe
       *
       * Subject to:
       *
       *  The tRD is required to be larger than 40 ns.
       *
       *  tRD  = [(STROBE_READ+1) – 2×(RELAX_READ+1)]/ipg_clk_freq.
       *  40 =  [(STROBE_READ+1) – 2×(RELAX_READ+1)]/ipg_clk_freq.
       *  STROBE_READ =  (40 * ipg_clk_freq) + 2×(RELAX_READ+1)) - 1
       *
       */

     read =  (((uint64_t)(40LL * ipg_freq_hz) + 1000000000LL) /
                      1000000000LL + 2 * (relax_read + 1)) - 1;

     modifyreg32(IMXRT_OCOTP_TIMING,
                 OCOTP_TIMING_WAIT_MASK | OCOTP_TIMING_STROBE_READ_MASK |
                 OCOTP_TIMING_RELAX_MASK | OCOTP_TIMING_STROBE_PROG_MASK,
                 OCOTP_TIMING_WAIT(wait) | OCOTP_TIMING_STROBE_READ(read) |
                 OCOTP_TIMING_RELAX(relax) | OCOTP_TIMING_STROBE_PROG(prog));

     modifyreg32(IMXRT_OCOTP_TIMING2,
                  OCOTP_TIMING2_RELAX_READ_MASK |
                  OCOTP_TIMING2_RELAX_PROG_MASK,
                  OCOTP_TIMING2_RELAX_READ(relax_read) |
                  OCOTP_TIMING2_RELAX_PROG(relax_prog));
    }
}

static int imxrt_ocotp_wait_for_completion(uint32_t timeout_ms)
{
  clock_t timeout;
  clock_t start;

  /* Get the timeout value */

  timeout = MSEC2TICK(timeout_ms);

  start = clock_systime_ticks();

  while (getreg32(IMXRT_OCOTP_CTRL) & OCOTP_CTRL_BUSY)
    {
      /* If a timeout is specified check for timeout */

      if (timeout_ms && clock_systime_ticks() - start >= timeout)
        {
          return -ETIME;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ocotp_reload
 *
 * Description:
 *   Reload the Shadow from OTP.
 *
 ****************************************************************************/

int imxrt_ocotp_reload()
{
  int ret;

  ret = imxrt_ocotp_wait_for_completion(OCOTP_OPT_TIMEOUT_MS);
  if (ret == OK)
    {
      imxrt_ocotp_reset_errors();
      imxrt_ocotp_initialize();

      putreg32(OCOTP_CTRL_RELOAD_SHADOWS, IMXRT_OCOTP_CTRL_SET);
      ret = imxrt_ocotp_wait_for_completion(OCOTP_OPT_TIMEOUT_MS);
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_ocotp_read
 *
 * Description:
 *   Read one value from the OTP.
 *
 * Input Parameters:
 *   otp_index - otp index (0-63)
 *   data      - a pointer to store the retrieved data
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int imxrt_ocotp_read(uint32_t otp_index, uint32_t *data)
{
  int ret;

  ret = imxrt_ocotp_wait_for_completion(OCOTP_OPT_TIMEOUT_MS);
  if (ret == OK)
    {
      imxrt_ocotp_reset_errors();
      imxrt_ocotp_initialize();

      putreg32(OCOTP_CTRL_ADDR_MASK, IMXRT_OCOTP_CTRL_CLR);
      putreg32(OCOTP_CTRL_ADDR(otp_index), IMXRT_OCOTP_CTRL_SET);
      putreg32(OCOTP_READ_CTRL_READ_FUSE, IMXRT_OCOTP_READ_CTRL);

      ret = imxrt_ocotp_wait_for_completion(OCOTP_OPT_TIMEOUT_MS);
      if (ret == OK)
        {
          if ((getreg32(IMXRT_OCOTP_CTRL) & OCOTP_CTRL_ERROR) != 0)
            {
              ret = -EIO;
            }
          else
            {
              *data = getreg32(IMXRT_OCOTP_READ_FUSE_DATA);
              ret = 0;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_ocotp_write
 *
 * Description:
 *   Write one value to the OTP.
 *
 * Input Parameters:
 *   otp_index - otp index (0-63)
 *   data      - data to write to OTP
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 *
 ****************************************************************************/

int imxrt_ocotp_write(uint32_t otp_index, uint32_t data)
{
  int ret;
  ret = imxrt_ocotp_wait_for_completion(OCOTP_OPT_TIMEOUT_MS);
  if (ret == OK)
    {
      imxrt_ocotp_reset_errors();
      imxrt_ocotp_initialize();

      putreg32(OCOTP_CTRL_ADDR_MASK | OCOTP_CTRL_WR_UNLOCK_MASK,
               IMXRT_OCOTP_CTRL_CLR);
      putreg32(OCOTP_CTRL_ADDR(otp_index) | OCOTP_CTRL_WR_UNLOCK,
               IMXRT_OCOTP_CTRL_SET);
      putreg32(data, IMXRT_OCOTP_DATA);

      ret = imxrt_ocotp_wait_for_completion(OCOTP_OPT_TIMEOUT_MS);
      if (ret == OK)
        {
          if ((getreg32(IMXRT_OCOTP_CTRL) & OCOTP_CTRL_ERROR) != 0)
            {
              ret = -EIO;
            }
          else
            {
              ret = imxrt_ocotp_reload();
            }
        }
    }

  return ret;
}
