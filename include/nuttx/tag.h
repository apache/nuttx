/****************************************************************************
 * include/nuttx/tag.h
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

#ifndef __INCLUDE_NUTTX_TAG_H
#define __INCLUDE_NUTTX_TAG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <string.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Unified tag enumeration -- single source of truth for all tags.
 * Auto-numbered; add new entries alphabetically before NXTAG_KERNEL_LAST.
 */

enum nxtag_e
{
  NXTAG_ANALOG,
  NXTAG_AUDIO,
  NXTAG_BATTERY,
  NXTAG_BINFMT,
  NXTAG_CAN,
  NXTAG_CAPTURE,
  NXTAG_CONTACTLESS,
  NXTAG_CRYPTO,
  NXTAG_DMA,
  NXTAG_FS,
  NXTAG_GPIO,
  NXTAG_GRAPHICS,
  NXTAG_HWTRACING,
  NXTAG_I2C,
  NXTAG_I2S,
  NXTAG_I3C,
  NXTAG_INPUT,
  NXTAG_IPC,
  NXTAG_IRQ,
  NXTAG_LCD,
  NXTAG_LEDS,
  NXTAG_LIB,
  NXTAG_MEMCARD,
  NXTAG_MM,
  NXTAG_MOTOR,
  NXTAG_NET,
  NXTAG_PAGING,
  NXTAG_PCI,
  NXTAG_POWER,
  NXTAG_PTP,
  NXTAG_PWM,
  NXTAG_RC,
  NXTAG_REGMAP,
  NXTAG_RESET,
  NXTAG_RMT,
  NXTAG_RPMSG,
  NXTAG_RPTUN,
  NXTAG_RTC,
  NXTAG_SAFETY,
  NXTAG_SCHED,
  NXTAG_SENSORS,
  NXTAG_SPI,
  NXTAG_STEPPER,
  NXTAG_SYSCALL,
  NXTAG_THERMAL,
  NXTAG_TIMER,
  NXTAG_USB,
  NXTAG_VHOST,
  NXTAG_VIDEO,
  NXTAG_VIRTIO,
  NXTAG_WATCHDOG,
  NXTAG_WIRELESS,
  NXTAG_KERNEL_MAX,

  NXTAG_USER_BASE = 128,
  NXTAG_MAX = 255
};

/* Bitmask helpers for tag enable/disable masks.
 * A mask is an array of uint8_t with NXTAG_MASK_SIZE bytes.
 */

#define NXTAG_MASK_SIZE             ((NXTAG_MAX + 7) / 8)
#define NXTAG_MASK_SET(mask, tag)   ((mask)[(tag) / 8] |= (1u << ((tag) % 8)))
#define NXTAG_MASK_CLR(mask, tag)   ((mask)[(tag) / 8] &= ~(1u << ((tag) % 8)))
#define NXTAG_MASK_ISSET(mask, tag) ((mask)[(tag) / 8] & (1u << ((tag) % 8)))
#define NXTAG_MASK_ZERO(mask)       memset((mask), 0, NXTAG_MASK_SIZE)
#define NXTAG_MASK_FILL(mask)       memset((mask), 0xff, NXTAG_MASK_SIZE)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: nxtag_get_name
 *
 * Description:
 *   Get the human-readable name for a tag (kernel or user).
 *
 * Input Parameters:
 *   tag - Tag value (0..NXTAG_MAX-1)
 *
 * Returned Value:
 *   Pointer to name string, or NULL if not registered.
 *
 ****************************************************************************/

FAR const char *nxtag_get_name(uint8_t tag);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TAG_H */
