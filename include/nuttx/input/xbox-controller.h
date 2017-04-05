/************************************************************************************
 * include/nuttx/input/xbox-controller.h
 *
 *   Copyright (C) 2016 Brian Webb.
 *   Author: Brian Webb <webbbn@gmail.com>
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_XBOX_CONTROLLER_H
#define __INCLUDE_NUTTX_INPUT_XBOX_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type defines the button data report from the controller. */

struct xbox_controller_buttonstate_s
{
  bool guide : 1;
  bool sync : 1;
  bool start : 1;
  bool back : 1;
  bool a : 1;
  bool b : 1;
  bool x : 1;
  bool y : 1;
  bool dpad_up : 1;
  bool dpad_down : 1;
  bool dpad_left : 1;
  bool dpad_right : 1;
  bool bumper_left : 1;
  bool bumper_right : 1;
  bool stick_click_left : 1;
  bool stick_click_right : 1;
  int16_t stick_left_x;
  int16_t stick_left_y;
  int16_t stick_right_x;
  int16_t stick_right_y;
  int16_t trigger_left;
  int16_t trigger_right;
};

/* The supported IOCTL commands. */

enum
{
  XBOX_CONTROLLER_IOCTL_RUMBLE
} xbox_controller_iotcl_cmds;

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_XBOX_CONTROLLER_H */

