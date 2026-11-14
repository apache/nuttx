/****************************************************************************
 * include/nuttx/timers/ptp_pps.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_PTP_PPS_H
#define __INCLUDE_NUTTX_TIMERS_PTP_PPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/timers/ptp_clock.h>

#ifdef CONFIG_PTP_PPS

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* PPS disciplined software PTP clock.
 *
 * The clock is built on a free running, 32-bit, up counting hardware
 * counter (for example a timer clocked by the CPU clock).  The time of the
 * clock is computed from the counter with a 64-bit fixed point conversion
 * whose rate is changed by clock_adjtime(ADJ_FREQUENCY), so the clock can
 * be disciplined with a resolution better than 1 ppb.
 *
 * External time stamp inputs latch the counter in hardware when an edge is
 * seen (for example a timer input capture channel wired to the PPS output
 * of a GNSS receiver).  The lower half reports the latched value with
 * ptp_pps_capture() from its interrupt handler.  Because the counter is
 * latched by hardware, the time stamp does not depend on the interrupt
 * latency at all: the interrupt only has to be serviced before the
 * counter wraps around.
 *
 *   PPS pin --> [timer input capture] --CCR--> ptp_pps_capture()
 *                                                  |
 *                     counter -> ns conversion <---+
 *                                                  |
 *               /dev/ptpX read(): struct ptp_extts_event
 */

struct ptp_pps_lowerhalf_s;

struct ptp_pps_ops_s
{
  /**************************************************************************
   * Name: read
   *
   * Description:
   *   Return the current value of the free running counter.  The counter
   *   must count up at the frequency given in ptp_pps_lowerhalf_s::freq and
   *   wrap around from 0xffffffff to 0.  This is called with interrupts
   *   disabled and must be fast.
   *
   **************************************************************************/

  CODE uint32_t (*read)(FAR struct ptp_pps_lowerhalf_s *lower);

  /**************************************************************************
   * Name: enable
   *
   * Description:
   *   Enable or disable the capture of an external time stamp input.
   *
   * Input Parameters:
   *   lower - The lower half instance.
   *   index - The capture input, 0 .. n_ext_ts - 1.
   *   flags - PTP_RISING_EDGE and/or PTP_FALLING_EDGE.
   *   on    - true to enable the input, false to disable it.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.  A driver
   *   that cannot capture the requested edges returns -EOPNOTSUPP.
   *
   **************************************************************************/

  CODE int (*enable)(FAR struct ptp_pps_lowerhalf_s *lower,
                     unsigned int index, unsigned int flags, bool on);
};

struct ptp_pps_lowerhalf_s
{
  FAR const struct ptp_pps_ops_s *ops; /* Lower half operations */
  uint32_t freq;                       /* Nominal counter frequency (Hz) */
  uint8_t  n_ext_ts;                   /* Number of capture inputs */

  /* Private data of the upper half, set by ptp_pps_register() */

  FAR void *priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ptp_pps_register
 *
 * Description:
 *   Create a PTP clock on top of the counter of the lower half and register
 *   it as /dev/ptp<devno>.  The clock starts at zero; set it with
 *   clock_settime() or let the gnsspps application do it.
 *
 * Input Parameters:
 *   lower - The lower half instance.  It must persist as long as the
 *           driver persists.
 *   devno - The device number, the device is registered as /dev/ptp<devno>
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ptp_pps_register(FAR struct ptp_pps_lowerhalf_s *lower, int devno);

/****************************************************************************
 * Name: ptp_pps_capture
 *
 * Description:
 *   Report a counter value latched by a capture input.  The value is
 *   converted to the time of the clock and queued as a struct
 *   ptp_extts_event for the readers of /dev/ptpX.
 *
 *   This is normally called from the capture interrupt handler.  It uses
 *   OS services so it must not be called from a zero latency
 *   (CONFIG_ARCH_HIPRI_INTERRUPT) handler.
 *
 *   The capture must be reported less than 2^31 counter periods after it
 *   happened (about 10 seconds with a 200 MHz counter).
 *
 * Input Parameters:
 *   lower - The lower half instance.
 *   index - The capture input that latched the counter.
 *   flags - The edge that was captured, PTP_RISING_EDGE, PTP_FALLING_EDGE
 *           or 0 when unknown.
 *   count - The latched counter value.
 *
 ****************************************************************************/

void ptp_pps_capture(FAR struct ptp_pps_lowerhalf_s *lower,
                     unsigned int index, unsigned int flags,
                     uint32_t count);

#ifdef CONFIG_PTP_PPS_DUMMY
/****************************************************************************
 * Name: ptp_pps_dummy_initialize
 *
 * Description:
 *   Register a PPS disciplined clock with a simulated counter and a
 *   simulated PPS input, for testing.
 *
 ****************************************************************************/

int ptp_pps_dummy_initialize(int devno);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PTP_PPS */
#endif /* __INCLUDE_NUTTX_TIMERS_PTP_PPS_H */
