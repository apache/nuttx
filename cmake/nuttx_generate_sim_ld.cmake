# ##############################################################################
# cmake/nuttx_generate_sim_ld.cmake
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

# cmake-format: off
# C++ global objects are constructed before main get executed, but it isn't a
# good point for simulator because NuttX doesn't finish the kernel
# initialization yet.
# So we have to skip the standard facilities and do the construction by ourself.
# But how to achieve the goal?
# 1.Command linker generate the default script(-verbose)
# 2.Replace __init_array_start/__init_array_end with _sinit/_einit
# 3.Append __init_array_start = .; __init_array_end = .;
# Step 2 let nxtask_startup find objects need to construct
# Step 3 cheat the host there is no object to construct
# Note: the destructor can be fixed in the same way.
set(PROCESS_SIM_LD_SCRIPT
    [[
    #!/bin/sh
    original_ld="$1"
    target_ld="$2"
    cat $original_ld | \
    sed -e '/====/,/====/!d;//d' \
    -e '/__executable_start/s/$/PROVIDE(_stext = .);/' \
    -e 's/^\(\s\+\)\(\.init_array\)/\1\2 : { }\n\1.sinit/g' \
    -e 's/^\(\s\+\)\(\.fini_array\)/\1\2 : { }\n\1.einit/g' \
    -e 's/__init_array_start/_sinit/g' -e 's/__init_array_end/_einit/g' \
    -e 's/__fini_array_start/_sfini/g' -e 's/__fini_array_end/_efini/g' > "$target_ld"
    echo "__init_array_start = .; __init_array_end = .; __fini_array_start = .; __fini_array_end = .;" >> "$target_ld"
]])
# cmake-format: on

file(WRITE ${CMAKE_BINARY_DIR}/process_sim_ld_script.sh
     "${PROCESS_SIM_LD_SCRIPT}")
file(
  COPY ${CMAKE_BINARY_DIR}/process_sim_ld_script.sh
  DESTINATION ${CMAKE_BINARY_DIR}
  FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)

add_custom_command(
  OUTPUT nuttx.ld
  COMMAND
    ${CMAKE_C_COMPILER} ${CMAKE_EXE_LINKER_FLAGS}
    $<$<BOOL:${CONFIG_SIM_M32}>:-m32> -Wl,-verbose 2> /dev/null > nuttx-orig.ld
    || true
  COMMAND sh process_sim_ld_script.sh nuttx-orig.ld nuttx.ld
  COMMAND sed -i '/\\.data *:/i " ${CONFIG_SIM_CUSTOM_DATA_SECTION} " ' nuttx.ld
  COMMENT "Generating sim linker script nuttx.ld"
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
