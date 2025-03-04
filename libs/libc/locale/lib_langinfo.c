/****************************************************************************
 * libs/libc/locale/lib_langinfo.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdlib.h>
#include <langinfo.h>

#ifdef CONFIG_LIBC_LOCALE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nl_langinfo
 *
 * Description:
 *   locales are not supported by NuttX
 *
 ****************************************************************************/

FAR char *nl_langinfo(nl_item item)
{
  switch (item)
    {
      case CODESET:

        /* if current locale encode length are 1, the codeset are ASCII,
         * then we support utf-8
         */

        if (MB_CUR_MAX == 1)
          {
            return "ASCII";
          }

        return "UTF-8";
      case RADIXCHAR:
        return ".";
      case THOUSEP:
        return ",";
      case ABDAY_1:
        return "Sun";
      case ABDAY_2:
        return "Mon";
      case ABDAY_3:
        return "Tue";
      case ABDAY_4:
        return "Wed";
      case ABDAY_5:
        return "Thu";
      case ABDAY_6:
        return "Fri";
      case ABDAY_7:
        return "Sat";
      case DAY_1:
        return "Sunday";
      case DAY_2:
        return "Monday";
      case DAY_3:
        return "Tuesday";
      case DAY_4:
        return "Wednesday";
      case DAY_5:
        return "Thursday";
      case DAY_6:
        return "Friday";
      case DAY_7:
        return "Saturday";
      case ABMON_1:
        return "Jan";
      case ABMON_2:
        return "Feb";
      case ABMON_3:
        return "Mar";
      case ABMON_4:
        return "Apr";
      case ABMON_5:
        return "May";
      case ABMON_6:
        return "Jun";
      case ABMON_7:
        return "Jul";
      case ABMON_8:
        return "Aug";
      case ABMON_9:
        return "Sep";
      case ABMON_10:
        return "Oct";
      case ABMON_11:
        return "Nov";
      case ABMON_12:
        return "Dev";
      case MON_1:
        return "January";
      case MON_2:
        return "Feburary";
      case MON_3:
        return "March";
      case MON_4:
        return "April";
      case MON_5:
        return "May";
      case MON_6:
        return "June";
      case MON_7:
        return "July";
      case MON_8:
        return "August";
      case MON_9:
        return "September";
      case MON_10:
        return "October";
      case MON_11:
        return "November";
      case MON_12:
        return "December";
      case AM_STR:
        return "AM";
      case PM_STR:
        return "PM";
      case D_T_FMT:
        return "%a %b %e %H:%M:%S %Y";
      case D_FMT:
        return "%F";
      case T_FMT:
        return "%T";
      case T_FMT_AMPM:
        return "%I:%M:%S %p";
      case ERA:
        return "";
      case ERA_D_FMT:
        return "";
      case ALT_DIGITS:
        return "";
      case ERA_D_T_FMT:
        return "";
      case ERA_T_FMT:
        return "";
      case CRNCYSTR:
        return "";
      case YESEXPR:
        return "^[yY]";
      case NOEXPR:
        return "^[nN]";
      case YESSTR:
        return "yes";
      case NOSTR:
        return "no";
    }

  return "";
}

#endif
