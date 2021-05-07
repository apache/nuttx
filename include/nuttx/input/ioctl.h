/****************************************************************************
 * include/nuttx/input/ioctl.h
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

#ifndef __INCLUDE_NUTTX_INPUT_IOCTL_H
#define __INCLUDE_NUTTX_INPUT_IOCTL_H

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
