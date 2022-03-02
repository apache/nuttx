/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_freerun.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_FREERUN_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_FREERUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include "esp32s3_tim.h"

#ifdef CONFIG_ESP32S3_FREERUN

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The freerun client must allocate an instance of this structure and called
 * esp32s3_freerun_initialize() before using the freerun facilities.  The
 * client should not access the contents of this structure directly since
 * the contents are subject to change.
 */

struct esp32s3_freerun_s
{
  uint8_t chan;                  /* The timer/counter in use */
  uint32_t overflow;             /* Timer counter overflow */
  uint16_t resolution;           /* Timer resolution */
  uint64_t max_timeout;          /* Maximum timeout to overflow */
  struct esp32s3_tim_dev_s *tch; /* Handle returned by esp32s3_tim_init() */
};

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_freerun_initialize
 *
 * Description:
 *   Initialize the freerun timer wrapper.
 *
 * Input Parameters:
 *   freerun          - Caller allocated instance of the freerun
 *                      state structure
 *   chan             - Timer counter channel to be used.
 *   resolution       - The required resolution of the timer in units of
 *                      microseconds.  NOTE that the range is restricted to
 *                      the range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s3_freerun_initialize(struct esp32s3_freerun_s *freerun, int chan,
                               uint16_t resolution);

/****************************************************************************
 * Name: esp32s3_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun          - Caller allocated instance of the freerun state
 *                      structure.  This structure must have been previously
 *                      initialized via a call to
 *                      esp32s3_freerun_initialize();
 *   ts               - The location in which to return the time from the
 *                      free-running timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s3_freerun_counter(struct esp32s3_freerun_s *freerun,
                            struct timespec *ts);

/****************************************************************************
 * Name: esp32s3_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun          - Caller allocated instance of the freerun state
 *                      structure. This structure must have been previously
 *                      initialized via a call to
 *                      esp32s3_freerun_initialize();
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s3_freerun_uninitialize(struct esp32s3_freerun_s *freerun);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ESP32S3_FREERUN */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_FREERUN_H */
