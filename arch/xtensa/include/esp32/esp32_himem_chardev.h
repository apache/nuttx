/****************************************************************************
 * arch/xtensa/include/esp32/esp32_himem_chardev.h
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ARCH_XTENSA_INCLUDE_ESP32_ESP32_HIMEM_CHARDEV_H
#define __ARCH_XTENSA_INCLUDE_ESP32_ESP32_HIMEM_CHARDEV_H

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: himem_chardev_init
 *
 * Description:
 *   Himem_Cdev Initializes the operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns 0 on success and non-zero on failure.
 *
 ****************************************************************************/

int himem_chardev_init(void);

/****************************************************************************
 * Name: himem_chardev_exit
 *
 * Description:
 *   Himem_Cdev exits and releases the operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns 0 on success and non-zero on failure.
 *
 ****************************************************************************/

int himem_chardev_exit(void);

/****************************************************************************
 * Name: himem_chardev_register
 *
 * Description:
 *   Himem_Cdev indicates the registration operation.
 *
 * Input Parameters:
 *   name - Himem_Cdev indicates the registration name.
 *   size - Himem_Cdev registers the device size.
 *
 * Returned Value:
 *   Returns 0 on success and non-zero on failure.
 *
 ****************************************************************************/

int himem_chardev_register(char *name, size_t size);

/****************************************************************************
 * Name: himem_chardev_unregister
 *
 * Description:
 *   Himem_Cdev cancels the registration
 *
 * Input Parameters:
 *   name - Himem_Cdev indicates the registration name.
 *
 * Returned Value:
 *   Returns 0 on success and non-zero on failure.
 *
 ****************************************************************************/

int himem_chardev_unregister(char *name);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_XTENSA_INCLUDE_ESP32_ESP32_HIMEM_CHARDEV_H */
