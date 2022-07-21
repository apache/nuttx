/****************************************************************************
 * arch/arm/src/common/arm_semi_syslog.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <syscall.h>

#include "arm_internal.h"

#ifdef CONFIG_ARM_SEMIHOSTING_SYSLOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SEMI_SYSLOG_WRITEC  0x03
#define SEMI_SYSLOG_WRITE0  0x04
#define SEMI_SYSLOG_READC   0x07

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

int up_putc(int ch)
{
  smh_call(SEMI_SYSLOG_WRITEC, &ch);
  return ch;
}

/****************************************************************************
 * Name: up_puts
 *
 * Description:
 *   Output a string on the console
 *
 ****************************************************************************/

void up_puts(const char *str)
{
  smh_call(SEMI_SYSLOG_WRITE0, (char *)str);
}
#endif
