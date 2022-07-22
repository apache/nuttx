/****************************************************************************
 * arch/arm/src/rp2040/rp2040_ws2812.h
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

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_WS2812_H
#define __ARCH_ARM_SRC_RP2040_RP2040_WS2812_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_WS2812

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_ws2812_setup
 *
 * Description:
 *   Initialize and register the ws2812 driver.
 *
 * Input Parameters:
 *   Path to the ws2812 device  (e.g. "/dev/leds0")
 *   Port number for the ws2812 chain
 *   Pin for ws2812 power
 *   The number of pixels in the chain
 *   Whether ws2812s have white LEDs
 *
 * Returned Value:
 *   An opaque pointer that can be passed to rp2040_ws2812_teardown on
 *   success or NULL (with errno set) on failure
 ****************************************************************************/

FAR void * rp2040_ws2812_setup(FAR const char *path,
                               int             port,
                               int             power_pin,
                               uint16_t        pixel_count,
                               bool            has_white);

/****************************************************************************
 * Name: rp2040_ws2812_release
 *
 * Description:
 *   This function releases the internal memory structures created when
 *   a driver is opened.  It will fail with an error -EBUSY the driver
 *   is open when it is called.
 *
 * Input Parameters:
 *   driver      - Opaque pointer returned by rp2040_ws2812_setup.
 *
 * Returned Value:
 *   OK on success or an ERROR on failure
 *
 ****************************************************************************/

int rp2040_ws2812_release(FAR void * driver);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_WS2812_H */
