/****************************************************************************
 * include/nuttx/timers/ptp_clock_dummy.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_PTP_CLOCK_DUMMY_H
#define __INCLUDE_NUTTX_TIMERS_PTP_CLOCK_DUMMY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ptp_clock_dummy_initialize
 *
 * Description:
 *   This function will initialize dummy ptp device driver for test.
 *
 * Input Parameters:
 *   devno - The user specifies number of device. ex: /dev/ptpX.
 *
 * Returned Value:
 *   OK if the driver was successfully initialize; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int ptp_clock_dummy_initialize(int devno);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_TIMERS_PTP_CLOCK_DUMMY_H */
