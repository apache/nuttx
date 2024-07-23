/****************************************************************************
 * arch/arm64/src/imx9/imx9_usbdev.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_USBDEV_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_USBDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  IMX9_USBC1 = 0,
  IMX9_USBC2 = 1,
} imx9_usb_id_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: imx9_vbus_detect
 *
 * Description:
 *   Read the VBUS state from the USB OTG controller. This can be used
 *   to poll the VBUS state
 *
 * Input Parameters:
 *   id: IMX9_USBC1 or IMX9_USBC2
 *
 * Returned Value:
 *   true if VBUS is valid; false otherwise
 *
 ****************************************************************************/

bool imx9_vbus_detect(imx9_usb_id_t id);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_USBDEV_H */
