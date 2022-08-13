/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_usbserial.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_USBSERIAL_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_USBSERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/serial/serial.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uart_dev_t g_uart_usbserial;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_usbserial_write
 *
 * Description:
 *   Write one character through the USB serial.  Used mainly for early
 *   debugging.
 *
 ****************************************************************************/

void esp32s3_usbserial_write(char ch);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_USBSERIAL_H */
