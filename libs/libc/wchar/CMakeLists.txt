# ##############################################################################
# libs/libc/wchar/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

target_sources(
  c
  PRIVATE lib_wcscmp.c
          lib_wcslen.c
          lib_wmemchr.c
          lib_wmemcmp.c
          lib_wmemcpy.c
          lib_wmemmove.c
          lib_wmemset.c
          lib_btowc.c
          lib_mbrtowc.c
          lib_wctob.c
          lib_wcslcpy.c
          lib_wcsxfrm.c
          lib_wcrtomb.c
          lib_wcsftime.c
          lib_wcscoll.c
          lib_wcstol.c
          lib_wcstoll.c
          lib_wcstoul.c
          lib_wcstoull.c
          lib_wcstold.c
          lib_wcstof.c
          lib_wcstod.c
          lib_swprintf.c
          lib_mbsnrtowcs.c
          lib_wcsnrtombs.c
          lib_mbsinit.c
          lib_mbrlen.c
          lib_mbsrtowcs.c
          lib_wcsrtombs.c
          lib_wcscpy.c
          lib_wcscat.c
          lib_wcslcat.c
          lib_wcsncat.c
          lib_wcsrchr.c
          lib_wcschr.c
          lib_wcsncpy.c
          lib_wcsncmp.c
          lib_wcscspn.c
          lib_wcspbrk.c
          lib_wcsspn.c
          lib_wcsstr.c
          lib_wcstok.c
          lib_wcswcs.c
          lib_wcwidth.c
          lib_wcswidth.c)
