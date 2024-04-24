# ##############################################################################
# libs/libc/ctype/CMakeLists.txt
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
  PRIVATE lib_isalnum.c
          lib_isalpha.c
          lib_isascii.c
          lib_isblank.c
          lib_iscntrl.c
          lib_isdigit.c
          lib_isgraph.c
          lib_islower.c
          lib_isprint.c
          lib_ispunct.c
          lib_isspace.c
          lib_isupper.c
          lib_isxdigit.c
          lib_tolower.c
          lib_toupper.c
          lib_ctype.c)
