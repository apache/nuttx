# ##############################################################################
# libs/libxx/CMakeLists.txt
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

# Include the uClibc++ Make.defs file if selected.  If it is included, the
# uClibc++/Make.defs file will add its files to the source file list, add its
# DEPPATH info, and will add the appropriate paths to the VPATH variable
#
# Note that an error will occur if you select CONFIG_LIBXX_UCLIBCXX without
# installing the uClibc++ package.  This is intentional to let you know about
# the configuration problem.  Refer to the README.txt file in the NuttX uClibc++
# GIT repository for more information

if(CONFIG_HAVE_CXX)
  if(CONFIG_UCLIBCXX)
    include(uClibc++.cmake)
  elseif(CONFIG_LIBCXX)
    include(libcxx.cmake)
  elseif(CONFIG_LIBCXXMINI)
    include(libcxxmini.cmake)
  endif()

  if(CONFIG_LIBCXXABI)
    include(libcxxabi.cmake)
  endif()
endif()
