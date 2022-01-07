/****************************************************************************
 * include/nuttx/video/vnc.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_VNC_H
#define __INCLUDE_NUTTX_VIDEO_VNC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* These are the types of the function pointers used to support callouts when
 * mouse or keyboard inputs are received by the local VNC server from the
 * remove VNC client.  Notice that these callouts have arguments that match
 * the inputs to nx_kbdin() and nx_mousein().
 */

typedef CODE int (*vnc_mouseout_t)(FAR void *arg, int16_t x,
                                   int16_t y, uint8_t buttons);
typedef CODE int (*vnc_kbdout_t)(FAR void *arg, uint8_t nch,
                                 FAR const uint8_t *ch);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: vnc_fbinitialize
 *
 * Description:
 *   Initialize the VNC frame buffer driver.  The VNC frame buffer driver
 *   supports two initialization interfaces:  The standard up_fbinitialize()
 *   that will be called from the graphics layer and this special
 *   initialization function that can be used only by VNC aware OS logic.
 *
 *   The two initialization functions may be called separated or together in
 *   either order.  The difference is that standard up_fbinitialize(), if
 *   used by itself, will not have any remote mouse or keyboard inputs that
 *   are reported to the VNC framebuffer driver from the remote VNC client.
 *
 *   In the standard graphics architecture, the keyboard/mouse inputs are
 *   received by some application/board specific logic at the highest level
 *   in the architecture via input drivers.  The received keyboard/mouse
 *   input data must then be "injected" into NX where it can they can be
 *   assigned to the window that has focus.  They will eventually be
 *   received by the Window instances via NX callback methods.
 *
 *   NX is a middleware layer in the architecture, below the
 *   application/board specific logic but above the driver level logic.  The
 *   frame buffer driver, on the other hand lies at the very lowest level in
 *   the graphics architecture.  It cannot call upward into the application
 *   nor can it call upward into NX.  So, some other logic.
 *
 *   vnc_fbinitialize() provides an optional, alternative initialization
 *   function.  It is optional because it need not be called.  If it is not
 *   called, however, keyboard/mouse inputs from the remote VNC client will
 *   be lost.  By calling vnc_fbinitialize(), you can provide callout
 *   functions that can be received by logic higher in the architecture.
 *   These higher level callouts can then call nx_kbdin() or nx_mousein() on
 *   behalf of the VNC server.
 *
 *   See also nx_vnc_fbinitialize() in include/nuttx/nx/nx.h.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   kbdout - If non-NULL, then the pointed-to function will be called to
 *     handle all keyboard input as it is received.  This may be either raw,
 *     ASCII keypress input or encoded keyboard input as determined by
 *     CONFIG_VNCSERVER_KBDENCODE.  See include/nuttx/input/kbd_codec.h.
 *   mouseout - If non-NULL, then the pointed-to function will be called to
 *     handle all mouse input as it is received.
 *   arg - An opaque user provided argument that will be provided when the
 *    callouts are performed.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int vnc_fbinitialize(int display, vnc_kbdout_t kbdout,
                     vnc_mouseout_t mouseout, FAR void *arg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_VNC_H */
