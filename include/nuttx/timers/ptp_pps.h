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

#include <stdint.h>

#include <nuttx/timers/capture.h>

#ifdef CONFIG_PTP_PPS

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
 *   Create a PPS disciplined PTP clock on top of a capture lower half and
 *   register it as /dev/ptp<devno>.
 *
 *   The time of the clock is computed from the free running counter of the
 *   capture lower half (getcount) with a 64-bit fixed point conversion
 *   whose rate is changed by clock_adjtime(ADJ_FREQUENCY), so the clock
 *   can be disciplined with a resolution better than 1 ppb.  The external
 *   time stamp (EXTTS) input 0 of the clock is the capture channel: on
 *   each edge the counter value latched by the hardware (getcapture) is
 *   converted to the time of the clock and queued as a struct
 *   ptp_extts_event for the readers of /dev/ptpX.  Because the counter is
 *   latched by hardware, the time stamp does not depend on the interrupt
 *   latency.
 *
 *     PPS pin --> [timer input capture] --getcapture()--> EXTTS event
 *                         |                                    ^
 *                  free running counter --getcount()--> clock_gettime()
 *
 *   The lower half must implement start, getcount, getcapture, bind and
 *   unbind.  Its counter must count up at 'freq' Hz and wrap around from
 *   0xffffffff to 0, and the notification callback must be called less
 *   than 2^31 counter periods after the edge.  The capture is started by
 *   this function and the lower half must not be used by another driver.
 *
 *   The clock starts at zero; set it with clock_settime() or let the
 *   gnsspps application do it.
 *
 * Input Parameters:
 *   lower - The capture lower half.  It must persist as long as the
 *           driver persists.
 *   freq  - The frequency of the counter of the lower half, in Hz.
 *   devno - The device number, the device is registered as /dev/ptp<devno>
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ptp_pps_register(FAR struct cap_lowerhalf_s *lower, uint32_t freq,
                     int devno);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PTP_PPS */
#endif /* __INCLUDE_NUTTX_TIMERS_PTP_PPS_H */
