# ##############################################################################
# tools/espressif/espressif_burn_enc_key.cmake
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
#
# Include() from the main NuttX CMake configure when
# CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED. Defines the ``burn_enc_key`` target
# (no cmake -P). Expects NUTTX_DIR, CMAKE_BINARY_DIR, CMAKE_SOURCE_DIR.
#
# Serial port: ESPTOOL_PORT in the environment at build time (e.g.
# ESPTOOL_PORT=/dev/ttyUSB0). NOCHECK must also be set so the burn is explicit
# (see burn_flash_enc_key.py).
#
# ##############################################################################

find_program(ESPEFUSE espefuse espefuse.py)
if(NOT ESPEFUSE)
  message(FATAL_ERROR "espefuse.py not found (required for burn_enc_key)")
endif()

find_program(PYTHON3 python3)
if(NOT PYTHON3)
  message(FATAL_ERROR "python3 not found (required for burn_enc_key)")
endif()

set(BINARY_DIR "${CMAKE_BINARY_DIR}")
set(SOURCE_DIR "${CMAKE_SOURCE_DIR}")
include(${NUTTX_DIR}/tools/espressif/espressif_esptool_common.cmake)

if(NOT EXISTS "${FLASH_ENC_KEY_PATH}")
  message(
    FATAL_ERROR
      "burn_enc_key: flash encryption key file missing. "
      "Generate the encryption key using: espsecure.py generate_flash_encryption_key <key_name.bin>"
  )
endif()

if(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED)
  add_custom_target(
    burn_enc_key
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "burn_enc_key: device already encrypted (Kconfig); skipping E-Fuse burn."
    VERBATIM)
elseif(CONFIG_ESPRESSIF_EFUSE_VIRTUAL)
  add_custom_target(
    burn_enc_key
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "burn_enc_key: virtual E-Fuses enabled (Kconfig); skipping E-Fuse burn."
    VERBATIM)
else()
  if(CONFIG_ESPRESSIF_SECURE_FLASH_ENC_USE_HOST_KEY)
    set(_nuttx_burn_key_msg
        "Using host key: ${CONFIG_ESPRESSIF_SECURE_FLASH_ENC_HOST_KEY_NAME}")
  else()
    set(_nuttx_burn_key_msg
        "Using randomly generated key flow (see Kconfig / documentation).")
  endif()

  add_custom_target(
    burn_enc_key
    COMMAND
      bash -c
      "if [ -z \"$$ESPTOOL_PORT\" ]; then echo 'burn_enc_key: ESPTOOL_PORT is not set. Example: ESPTOOL_PORT=/dev/ttyUSB0 cmake --build . -t burn_enc_key'; exit 1; fi"
    COMMAND ${CMAKE_COMMAND} -E echo "${_nuttx_burn_key_msg}"
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "This operation is NOT REVERSIBLE. See flash encryption documentation."
    COMMAND
      ${CMAKE_COMMAND} -E env "NUTTX_ESPEFUSE=${ESPEFUSE}"
      "NUTTX_KEY=${FLASH_ENC_KEY_PATH}" -- ${PYTHON3}
      "${NUTTX_DIR}/tools/espressif/burn_flash_enc_key.py"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Burn flash encryption key to eFuses (espefuse.py)"
    VERBATIM)
  unset(_nuttx_burn_key_msg)
endif()
