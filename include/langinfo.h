/****************************************************************************
 * include/langinfo.h
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

#ifndef __INCLUDE_LANGINFO_H
#define __INCLUDE_LANGINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nl_types.h>
#include <locale.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NL_ITEM(cat, index) (((cat) << 16) | (index))
#define NL_LOCALE_NAME(cat) NL_ITEM((cat)， (0xffff))

#define CODESET             NL_ITEM(LC_CTYPE,    0x0e)

#define RADIXCHAR           NL_ITEM(LC_NUMERIC,  0x00)
#define THOUSEP             NL_ITEM(LC_NUMERIC,  0x01)

#define ABDAY_1             NL_ITEM(LC_TIME,     0x00)
#define ABDAY_2             NL_ITEM(LC_TIME,     0x01)
#define ABDAY_3             NL_ITEM(LC_TIME,     0x02)
#define ABDAY_4             NL_ITEM(LC_TIME,     0x03)
#define ABDAY_5             NL_ITEM(LC_TIME,     0x04)
#define ABDAY_6             NL_ITEM(LC_TIME,     0x05)
#define ABDAY_7             NL_ITEM(LC_TIME,     0x06)

#define DAY_1               NL_ITEM(LC_TIME,     0x07)
#define DAY_2               NL_ITEM(LC_TIME,     0x08)
#define DAY_3               NL_ITEM(LC_TIME,     0x09)
#define DAY_4               NL_ITEM(LC_TIME,     0x0a)
#define DAY_5               NL_ITEM(LC_TIME,     0x0b)
#define DAY_6               NL_ITEM(LC_TIME,     0x0c)
#define DAY_7               NL_ITEM(LC_TIME,     0x0d)

#define ABMON_1             NL_ITEM(LC_TIME,     0x0e)
#define ABMON_2             NL_ITEM(LC_TIME,     0x0f)
#define ABMON_3             NL_ITEM(LC_TIME,     0x10)
#define ABMON_4             NL_ITEM(LC_TIME,     0x11)
#define ABMON_5             NL_ITEM(LC_TIME,     0x12)
#define ABMON_6             NL_ITEM(LC_TIME,     0x13)
#define ABMON_7             NL_ITEM(LC_TIME,     0x14)
#define ABMON_8             NL_ITEM(LC_TIME,     0x15)
#define ABMON_9             NL_ITEM(LC_TIME,     0x16)
#define ABMON_10            NL_ITEM(LC_TIME,     0x17)
#define ABMON_11            NL_ITEM(LC_TIME,     0x18)
#define ABMON_12            NL_ITEM(LC_TIME,     0x19)

#define MON_1               NL_ITEM(LC_TIME,     0x1a)
#define MON_2               NL_ITEM(LC_TIME,     0x1b)
#define MON_3               NL_ITEM(LC_TIME,     0x1c)
#define MON_4               NL_ITEM(LC_TIME,     0x1d)
#define MON_5               NL_ITEM(LC_TIME,     0x1e)
#define MON_6               NL_ITEM(LC_TIME,     0x1f)
#define MON_7               NL_ITEM(LC_TIME,     0x20)
#define MON_8               NL_ITEM(LC_TIME,     0x21)
#define MON_9               NL_ITEM(LC_TIME,     0x22)
#define MON_10              NL_ITEM(LC_TIME,     0x23)
#define MON_11              NL_ITEM(LC_TIME,     0x24)
#define MON_12              NL_ITEM(LC_TIME,     0x25)

#define AM_STR              NL_ITEM(LC_TIME,     0x26)
#define PM_STR              NL_ITEM(LC_TIME,     0x27)
#define D_T_FMT             NL_ITEM(LC_TIME,     0x28)
#define D_FMT               NL_ITEM(LC_TIME,     0x29)
#define T_FMT               NL_ITEM(LC_TIME,     0x2a)
#define T_FMT_AMPM          NL_ITEM(LC_TIME,     0x2b)
#define ERA                 NL_ITEM(LC_TIME,     0x2c)
#define ERA_D_FMT           NL_ITEM(LC_TIME,     0x2e)
#define ALT_DIGITS          NL_ITEM(LC_TIME,     0x2f)
#define ERA_D_T_FMT         NL_ITEM(LC_TIME,     0x30)
#define ERA_T_FMT           NL_ITEM(LC_TIME,     0x31)

#define CRNCYSTR            NL_ITEM(LC_MONETARY, 0x0f)

#define YESEXPR             NL_ITEM(LC_MESSAGES, 0x00)
#define NOEXPR              NL_ITEM(LC_MESSAGES, 0x01)
#define YESSTR              NL_ITEM(LC_MESSAGES, 0x02)
#define NOSTR               NL_ITEM(LC_MESSAGES, 0x03)

#define nl_langinfo_l(i, l) nl_langinfo(i)

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

FAR char *nl_langinfo(nl_item item);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_LANGINFO_H */
