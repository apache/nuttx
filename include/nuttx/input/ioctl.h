/************************************************************************************
 * include/nuttx/input/ioctl.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_INPUT_IOCTL_H
#define __INCLUDE_NUTTX_INPUT_IOCTL_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _JOYIOC_MASK         (0x001f)
#define _JOYIOC_TYPE(cmd)    ((cmd) & ~_JOYIOC_MASK)
#define _JOYIOC_NBR(cmd)     ((cmd) & _JOYIOC_MASK)

/* Discrete Joystick (see nuttx/include/input/djoystick.h */

#define _DJOYBASE            (_JOYBASE | 0x0000)

#define _DJOYIOCVALID(c)     (_JOYIOC_MASK(c)==_DJOYBASE)
#define _DJOYIOC(nr)         _IOC(_DJOYBASE,nr)

/* Analog Joystick (see nuttx/include/input/ajoystick.h */

#define _AJOYBASE            (_JOYBASE | 0x0020)

#define _AJOYIOCVALID(c)     (_JOYIOC_MASK(c)==_AJOYBASE)
#define _AJOYIOC(nr)         _IOC(_AJOYBASE,nr)

/* Nunchuck Wii controller */

#define _NUNCKIOCBASE        (_JOYBASE | 0x0040)

#define _NUNCHUCKIOCVALID(c) (_IOC_TYPE(c)==_NUNCKIOCBASE)
#define _NUNCHUCKIOC(nr)     _IOC(_NUNCKIOCBASE,nr)

#endif /* __INCLUDE_NUTTX_INPUT_IOCTL_H */

