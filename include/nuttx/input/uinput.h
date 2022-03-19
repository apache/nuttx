/****************************************************************************
 * include/nuttx/input/uinput.h
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_UINPUT_H
#define __INCLUDE_NUTTX_INPUT_UINPUT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: uinput_touch_initialize
 *
 * Description:
 *   Initialized the uinput touchscreen device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_UINPUT_TOUCH
int uinput_touch_initialize(void);
#endif

/****************************************************************************
 * Name: uinput_keyboard_initialize
 *
 * Description:
 *   Initialized the uinput keyboard device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_UINPUT_KEYBOARD
int uinput_keyboard_initialize(void);
#endif

/****************************************************************************
 * Name: uinput_button_initialize
 *
 * Description:
 *   Initialized the uinput button device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_UINPUT_BUTTONS
int uinput_button_initialize(void);
#endif

#endif /* __INCLUDE_NUTTX_INPUT_UINPUT_H */
