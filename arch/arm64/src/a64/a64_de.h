/****************************************************************************
 * arch/arm64/src/a64/a64_de.h
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

#ifndef __ARCH_ARM64_SRC_A64_A64_DE_H
#define __ARCH_ARM64_SRC_A64_A64_DE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: a64_de_init
 *
 * Description:
 *   Initialize the Display Engine on the SoC.  Mixer 0 will be configured
 *   to stream pixel data to Timing Controller TCON0.  Should be called
 *   before any Display Engine operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ****************************************************************************/

int a64_de_init(void);

/****************************************************************************
 * Name: a64_de_blender_init
 *
 * Description:
 *   Initialize the UI Blender for Display Engine.  Should be called after
 *   a64_de_init() and before a64_de_ui_channel_init().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_de_blender_init(void);

/****************************************************************************
 * Name: a64_de_ui_channel_init
 *
 * Description:
 *   Initialize a UI Channel for Display Engine.  Display Engine will
 *   stream the pixel data from the Frame Buffer Memory (over DMA) to the
 *   UI Blender.  There are 3 UI Channels: Base UI Channel (Channel 1) and
 *   2 Overlay UI Channels (Channels 2 and 3).  Should be called after
 *   a64_de_blender_init() and before a64_de_enable().
 *
 * Input Parameters:
 *   channel - UI Channel Number: 1, 2 or 3
 *   fbmem   - Start of Frame Buffer Memory (address should be 32-bit),
 *             or NULL if this UI Channel should be disabled
 *   fblen   - Length of Frame Buffer Memory in bytes
 *   xres    - Horizontal resolution in pixel columns
 *   yres    - Vertical resolution in pixel rows
 *   xoffset - Horizontal offset in pixel columns
 *   yoffset - Vertical offset in pixel rows
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_de_ui_channel_init(uint8_t channel,
                           const void *fbmem,
                           size_t fblen,
                           uint16_t xres,
                           uint16_t yres,
                           uint16_t xoffset,
                           uint16_t yoffset);

/****************************************************************************
 * Name: a64_de_enable
 *
 * Description:
 *   Set the UI Blender Route, enable the Blender Pipes and enable the
 *   Display Engine.  Should be called after all 3 UI Channels have been
 *   initialized.
 *
 * Input Parameters:
 *   channels - Number of UI Channels to enable: 1 or 3
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_de_enable(uint8_t channels);

#endif /* __ARCH_ARM64_SRC_A64_A64_DE_H */
