# ##############################################################################
# boards/arm/rtl8721dx/pke8721daf/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
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

# The RTL8721Dx (Ameba WHC) port currently builds via the make-based build only.
# The CMake path does not yet wire up the vendor-SDK machinery (SDK fetch,
# libameba_fwlib.a/libameba_wifi.a, the image2 entry ameba_app_start.c, the SDK
# image2 linker-script generation, the NP-image build and the app.bin packaging)
# that boards/.../scripts/Make.defs -> arch/.../ameba_board.mk provides, so a
# CMake configure would not produce a flashable image.  Fail fast with a clear
# message instead of half-building.
message(
  FATAL_ERROR
    "pke8721daf (RTL8721Dx, Ameba WHC) currently supports the make build "
    "only; CMake is not yet supported.  Build with:\n"
    "  ./tools/configure.sh pke8721daf:nsh && make")

add_subdirectory(src)
