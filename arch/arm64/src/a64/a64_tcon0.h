/****************************************************************************
 * arch/arm64/src/a64/a64_tcon0.h
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

#ifndef __ARCH_ARM64_SRC_A64_A64_TCON0_H
#define __ARCH_ARM64_SRC_A64_A64_TCON0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: a64_tcon0_init
 *
 * Description:
 *   Initialize Timing Controller TCON0 to stream pixel data from Display
 *   Engine to MIPI Display Serial Interface. Should be called before
 *   enabling the MIPI DSI Block on the SoC.
 *
 * Input Parameters:
 *   panel_width  - LCD Panel Width (pixels)
 *   panel_height - LCD Panel Height (pixels)
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_tcon0_init(uint16_t panel_width, uint16_t panel_height);

#endif /* __ARCH_ARM64_SRC_A64_A64_TCON0_H */
