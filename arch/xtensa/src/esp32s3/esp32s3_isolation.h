/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_isolation.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ISOLATION_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ISOLATION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef CONFIG_BUILD_FLAT

/****************************************************************************
 * Name: esp32s3_isolation_revoke_peripherals
 *
 * Description:
 *   Revoke the unprivileged world's access to every peripheral.  A user
 *   task reaches a peripheral only through a system call, so WORLD1 has no
 *   business addressing one directly.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_isolation_revoke_peripherals(void);

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Name: esp32s3_isolation_worlds
 *
 * Description:
 *   Give the unprivileged world its own vector table and tell the World
 *   Controller which kernel vectors return the CPU to the privileged world.
 *   A protected build does the equivalent from esp32s3_userspace.c, where
 *   the table belongs to the user image instead.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_isolation_worlds(void);

/****************************************************************************
 * Name: esp32s3_isolation_permissions
 *
 * Description:
 *   Program the permission control for a kernel build: what the
 *   unprivileged world may reach, which is the WORLD1 vector table and
 *   nothing else in internal memory, and arm the violation monitors.  This
 *   is the kernel-build counterpart to configure_mpu() in
 *   esp32s3_userspace.c, which serves the protected user image.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_isolation_permissions(void);
#endif

/****************************************************************************
 * Name: esp32s3_pmsirqinitialize
 *
 * Description:
 *   Initialize interrupt handler for the PMS violation ISR.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_pmsirqinitialize(void);

#else
#  define esp32s3_pmsirqinitialize()
#endif /* !CONFIG_BUILD_FLAT */

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ISOLATION_H */
