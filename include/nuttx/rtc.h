/****************************************************************************
 * include/nuttx/rtc.h
 *
 *   Copyright(C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_RTC_H
#define __INCLUDE_NUTTX_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <time.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Access macros ************************************************************/

/* Clock manipulation macros ************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The type of the RTC callback function */

typedef void (*rtc_handler_t)(FAR void *arg);

/* The RTC vtable */

struct rtc_dev_s;
struct rtc_ops_s
{
  int (*trigger)(FAR struct rtc_dev_s *dev, FAR void *arg);
  int (*add)(FAR struct rtc_dev_s *dev, FAR void *arg, clock_t period);
  int (*set)(FAR struct rtc_dev_s *dev, FAR void *arg, clock_t period);
  int (*clear)(FAR struct rtc_dev_s *dev, FAR void *arg);
  clock_t (*remainder)(FAR struct rtc_dev_s *dev, FAR void *arg);
  clock_t (*overrun)(FAR struct rtc_dev_s *dev, FAR void *arg);
  int (*exec)(FAR struct rtc_dev_s *dev, clock_t timeout);
};

/* RTC private data.  This structure only defines the initial fields of the
 * structure visible to the SPI client.  The specific implementation may 
 * add additional, device specific fields
 */

struct rtc_dev_s
{
  FAR const struct rtc_ops_s *ops;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_rtcinitialize
 *
 * Description:
 *   Initialize the RTC interface. This function may be called to obtain
 *   multiple instances of the interface
 *
 * Returned Value:
 *   Valid RTC device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

EXTERN FAR struct rtc_dev_s *up_rtcinitialize(void);


#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_RTC_H */
