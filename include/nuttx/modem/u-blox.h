/****************************************************************************
 * include/nuttx/modem/u-blox.h
 *
 *   Copyright (C) 2016 Vladimir Komendantskiy. All rights reserved.
 *   Author: Vladimir Komendantskiy <vladimir@moixaenergy.com>
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

#ifndef __INCLUDE_NUTTX_MODEM_U_BLOX_H
#define __INCLUDE_NUTTX_MODEM_U_BLOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>

#include <nuttx/modem/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* See IOCTL definitions in include/nuttx/modem/ioctl.h */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ubxmdm_regval
{
  FAR char name[3];
  bool val;
};

struct ubxmdm_status
{
  bool on;
  int register_values_size;
  FAR struct ubxmdm_regval* register_values;
};

struct ubxmdm_lower;

struct ubxmdm_ops
{
  CODE int (*poweron)  (FAR struct ubxmdm_lower* lower);
  CODE int (*poweroff) (FAR struct ubxmdm_lower* lower);
  CODE int (*reset)    (FAR struct ubxmdm_lower* lower);
  CODE int (*getstatus)(FAR struct ubxmdm_lower* lower,
                        FAR struct ubxmdm_status* status);
  CODE int (*ioctl)    (FAR struct ubxmdm_lower* lower,
                        int cmd,
                        unsigned long arg);
};

struct ubxmdm_lower
{
  FAR const struct ubxmdm_ops *ops;
};

/****************************************************************************
 * Public Functions
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
 * "Upper-Half" Timer Driver Interfaces
 ****************************************************************************/

FAR void *ubxmdm_register(FAR const char *path,
                          FAR struct ubxmdm_lower *lower);

void ubxmdm_unregister(FAR void *handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MODEM_U_BLOX_H */
