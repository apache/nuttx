/****************************************************************************
 * arch/arm64/include/imx9/imx9_ahab.h
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

#ifndef __ARCH_ARM64_INCLUDE_IMX9_IMX9_AHAB_H
#define __ARCH_ARM64_INCLUDE_IMX9_IMX9_AHAB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <imx_container.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ahab_auth_release
 *
 * Description:
 *   Release the current container used by ELE for AHAB.
 *
 * Returned Value:
 *   Zero (OK) is returned for success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int ahab_auth_release(void);

/****************************************************************************
 * Name: ahab_read_auth_image
 *
 * Description:
 *   Copy to RAM, then authenticate the image_index image of the container.
 *
 * Input Parameters:
 *   container - Pointer to a copy of the AHAB container header in RAM.
 *   image_index - The index of the image in the current container.
 *   container_offset - Offset of the container in the boot device.
 *
 * Returned Value:
 *   NULL pointer for failure, otherwise a pointer to the beginning of the
 *   authenticated, image's header.
 *
 ****************************************************************************/

struct boot_img_hdr *ahab_read_auth_image(struct container_hdr *container,
                                          int image_index,
                                          unsigned long container_offset);

#endif /* __ARCH_ARM64_INCLUDE_IMX9_IMX9_AHAB_H */
