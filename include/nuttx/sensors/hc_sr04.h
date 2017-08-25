/****************************************************************************
 * include/nuttx/sensors/hc_sr04.h
 *
 *   Copyright (C) 2017 Greg Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTT_SENSORS_HCSR04_H
#define __INCLUDE_NUTT_SENSORS_HCSR04_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Interrupt configuration data structure */

struct hcsr04_config_s
{
  CODE int  (*irq_attach)(FAR struct hcsr04_config_s * state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR const struct hcsr04_config_s *state,
                          bool enable);
  CODE void (*irq_clear)(FAR const struct hcsr04_config_s *state);
  CODE void (*irq_setmode)(FAR struct hcsr04_config_s *state, bool risemode);
  CODE void (*set_trigger)(FAR const struct hcsr04_config_s *state, bool on);
  CODE int64_t (*get_clock)(FAR const struct hcsr04_config_s *state);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int hcsr04_register(FAR const char *devpath,
                    FAR struct hcsr04_config_s *config);

#endif /* __INCLUDE_NUTT_SENSORS_HCSR04_H */
