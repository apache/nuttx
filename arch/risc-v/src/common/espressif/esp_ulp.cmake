# ##############################################################################
# arch/risc-v/src/common/espressif/esp_ulp.cmake
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

if(CONFIG_ESPRESSIF_USE_LP_CORE)

  # ############################################################################
  # Path variables for ULP build
  # ############################################################################

  if(NOT CHIP_SERIES)
    string(REPLACE "\"" "" CHIP_SERIES "${CONFIG_ESPRESSIF_CHIP_SERIES}")
  endif()

  set(ARCH_SRCDIR ${NUTTX_DIR}/arch/${CONFIG_ARCH}/src)
  set(CHIP_DIR ${NUTTX_CHIP_ABS_DIR})
  set(ULP_ARCH_FOLDER ${CHIP_DIR}/ulp)
  set(ULP_FOLDER ${ULP_APP_FOLDER}/ulp)

  if(NOT ULP_APP_NAME)
    set(ULP_APP_NAME ${ULP_APP_FOLDER})
  endif()

  string(MAKE_C_IDENTIFIER "${ULP_APP_NAME}" _ulp_app_id)
  set(ULP_BIN_TARGET ${_ulp_app_id}_ulp_bin)
  set(ULP_PREPARE_TARGET ${_ulp_app_id}_ulp_prepare)
  set(ULP_FIRMWARE_TARGET ${_ulp_app_id}_ulp_firmware)

  if(DEFINED NUTTX_BOARD_DIR)
    set(ULP_BOARD_SCRIPTS_DIR ${NUTTX_BOARD_DIR}/../common/scripts)
  else()
    set(ULP_BOARD_SCRIPTS_DIR
        ${NUTTX_DIR}/boards/${CONFIG_ARCH}/${CHIP_SERIES}/common/scripts)
  endif()
  get_filename_component(ULP_BOARD_SCRIPTS_DIR ${ULP_BOARD_SCRIPTS_DIR}
                         ABSOLUTE)

  set(ULP_CODE_HEADER ${ULP_FOLDER}/ulp_code.h)
  set(ULP_VAR_MAP_HEADER ${ULP_ARCH_FOLDER}/ulp_var_map.h)
  set(ULP_VARS_HEADER ${ULP_ARCH_FOLDER}/ulp_vars.h)
  set(ULP_ALIASES_LD ${ULP_BOARD_SCRIPTS_DIR}/ulp_aliases.ld)
  set(ULP_MAIN_HEADER ${ULP_FOLDER}/ulp_main.h)
  set(ULP_MAIN_LD ${ULP_FOLDER}/ulp_main.ld)
  set(ULP_LOCKFILE ${ULP_VAR_MAP_HEADER}.lock)
  set(ULP_PREFIX ${ULP_APP_NAME}_)
  set(ULP_BASE 0)

  if(NOT ESP_HAL_3RDPARTY_REPO)
    get_filename_component(
      ESP_HAL_3RDPARTY_REPO
      "${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH}/src/common/espressif/esp-hal-3rdparty"
      REALPATH)
  endif()

  set(HAL ${ESP_HAL_3RDPARTY_REPO})

  set(ULP_LPCORE_SECTIONS_LD
      ${ULP_BOARD_SCRIPTS_DIR}/${CHIP_SERIES}_lpcore_sections.ld)
  set(ULP_SECTIONS_LD ${ULP_FOLDER}/ulp_sections.ld)
  set(ULP_MAPGEN_TOOL ${HAL}/components/ulp/esp32ulp_mapgen.py)

  find_program(XXD_PROGRAM xxd REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  get_filename_component(_ulp_toolchain_bin ${CMAKE_OBJCOPY} DIRECTORY)
  find_program(ULP_READELF readelf HINTS ${_ulp_toolchain_bin} REQUIRED)

  # ############################################################################
  # Include paths
  # ############################################################################

  set(ULP_INCLUDE_DIRS
      ${HAL}
      ${ULP_ARCH_FOLDER}
      ${HAL}/components/esp_common/include
      ${HAL}/components/esp_hal_ana_conv/include
      ${HAL}/components/esp_hal_ana_conv/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_clock/include
      ${HAL}/components/esp_hal_clock/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_gpio/include
      ${HAL}/components/esp_hal_gpio/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_gpspi/include
      ${HAL}/components/esp_hal_gpspi/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_i2c/include
      ${HAL}/components/esp_hal_i2c/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_pmu/include
      ${HAL}/components/esp_hal_pmu/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_rtc_timer/include
      ${HAL}/components/esp_hal_rtc_timer/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_uart/include
      ${HAL}/components/esp_hal_uart/${CHIP_SERIES}/include
      ${HAL}/components/esp_hw_support/include
      ${HAL}/components/esp_hw_support/port/${CHIP_SERIES}/include
      ${HAL}/components/esp_hw_support/port/${CHIP_SERIES}/private_include
      ${HAL}/components/esp_rom
      ${HAL}/components/esp_rom/include
      ${HAL}/components/esp_rom/${CHIP_SERIES}
      ${HAL}/components/esp_rom/${CHIP_SERIES}/include
      ${HAL}/components/esp_rom/${CHIP_SERIES}/include/${CHIP_SERIES}
      ${HAL}/components/hal/include
      ${HAL}/components/hal/platform_port/include
      ${HAL}/components/log
      ${HAL}/components/log/include
      ${HAL}/components/riscv/include
      ${HAL}/components/soc
      ${HAL}/components/soc/include
      ${HAL}/components/soc/${CHIP_SERIES}
      ${HAL}/components/soc/${CHIP_SERIES}/include
      ${HAL}/components/hal/${CHIP_SERIES}/include
      ${HAL}/components/soc/${CHIP_SERIES}/register
      ${HAL}/components/soc/${CHIP_SERIES}/register/soc
      ${HAL}/components/ulp
      ${HAL}/components/ulp/ulp_common
      ${HAL}/components/ulp/ulp_common/include
      ${HAL}/components/ulp/lp_core
      ${HAL}/components/ulp/lp_core/lp_core
      ${HAL}/components/ulp/lp_core/include
      ${HAL}/components/ulp/lp_core/shared
      ${HAL}/components/ulp/lp_core/lp_core/include
      ${HAL}/components/ulp/lp_core/shared/include
      ${HAL}/components/upper_hal_uart/include
      ${HAL}/nuttx/include
      ${HAL}/nuttx/${CHIP_SERIES}/include
      ${HAL}/nuttx/src/components/esp_driver_uart/include)

  if(CONFIG_ARCH_CHIP_ESP32P4)
    if(NOT CONFIG_ESP32P4_SELECTS_REV_LESS_V3)
      list(APPEND ULP_INCLUDE_DIRS
           ${HAL}/components/soc/${CHIP_SERIES}/register/hw_ver3
           ${HAL}/components/soc/${CHIP_SERIES}/register/soc/)
    else()
      list(APPEND ULP_INCLUDE_DIRS
           ${HAL}/components/soc/${CHIP_SERIES}/register/hw_ver1
           ${HAL}/components/soc/${CHIP_SERIES}/register/soc/)
    endif()
    list(
      APPEND
      ULP_INCLUDE_DIRS
      ${HAL}/components/esp_adc/include
      ${HAL}/components/esp_adc/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_i2s/include
      ${HAL}/components/esp_hal_i2s/${CHIP_SERIES}/include
      ${HAL}/components/esp_hal_touch_sens/include
      ${HAL}/components/esp_hal_touch_sens/${CHIP_SERIES}/include
      ${HAL}/components/esp_system/ld
      ${HAL}/components/upper_hal_i2s/include)
  endif()

  # ############################################################################
  # Linker scripts
  # ############################################################################

  set(ULP_LDSCRIPTS
      -T${HAL}/components/soc/${CHIP_SERIES}/ld/${CHIP_SERIES}.peripherals.ld
      -T${ULP_SECTIONS_LD})

  if(CONFIG_ARCH_CHIP_ESP32P4)
    list(
      APPEND
      ULP_LDSCRIPTS
      -T${HAL}/components/esp_rom/${CHIP_SERIES}/ld/${CHIP_SERIES}lp.rom.ld
      -T${HAL}/components/esp_rom/${CHIP_SERIES}/ld/${CHIP_SERIES}lp.rom.newlib.ld
      -T${HAL}/components/esp_rom/${CHIP_SERIES}/ld/${CHIP_SERIES}lp.rom.version.ld
      -T${HAL}/components/esp_rom/${CHIP_SERIES}/ld/${CHIP_SERIES}lp.rom.api.ld)
  endif()

  # ############################################################################
  # HAL / LP-core sources
  # ############################################################################

  set(ULP_ASOURCES
      ${HAL}/components/ulp/lp_core/lp_core/port/${CHIP_SERIES}/vector_table.S
      ${HAL}/components/ulp/lp_core/lp_core/start.S
      ${HAL}/components/ulp/lp_core/lp_core/vector.S)

  set(ULP_CSOURCES
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_spi.c
      ${HAL}/components/ulp/lp_core/shared/ulp_lp_core_memory_shared.c
      ${HAL}/components/ulp/lp_core/shared/ulp_lp_core_lp_uart_shared.c
      ${HAL}/components/ulp/lp_core/shared/ulp_lp_core_lp_timer_shared.c
      ${HAL}/components/esp_hal_uart/uart_hal_iram.c
      ${HAL}/components/esp_hal_uart/uart_hal.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_i2c.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_startup.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_utils.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_uart.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_print.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_panic.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_interrupt.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_ubsan.c
      ${HAL}/components/ulp/lp_core/shared/ulp_lp_core_lp_adc_shared.c
      ${HAL}/components/ulp/lp_core/shared/ulp_lp_core_lp_vad_shared.c
      ${HAL}/components/ulp/lp_core/shared/ulp_lp_core_critical_section_shared.c
  )

  if(CONFIG_ARCH_CHIP_ESP32P4)
    list(
      APPEND ULP_CSOURCES ${HAL}/components/ulp/lp_core/lp_core/lp_core_touch.c
      ${HAL}/components/ulp/lp_core/lp_core/port/lp_core_mailbox_impl_hw.c
      ${HAL}/components/ulp/lp_core/lp_core/lp_core_mailbox.c)
  endif()

  # ############################################################################
  # Shared chip-level stubs
  # ############################################################################

  if(NOT TARGET esp_ulp_shared_stubs)
    set(_ESP_ULP_BASH_WRITE_VAR_MAP_INIT
        [=[printf '%s\n' '#include "nuttx/symtab.h"' '#include "ulp/ulp_vars.h"' '' 'struct ulp_var_map_s' '{' '  struct symtab_s sym;' '  size_t size;' '};' '' 'struct ulp_var_map_s ulp_var_map[] =' '{ };' > "@ULP_VAR_MAP_HEADER@"]=]
    )
    string(REPLACE "@ULP_VAR_MAP_HEADER@" "${ULP_VAR_MAP_HEADER}"
                   _ESP_ULP_BASH_WRITE_VAR_MAP_INIT
                   "${_ESP_ULP_BASH_WRITE_VAR_MAP_INIT}")

    add_custom_command(
      OUTPUT ${ULP_VARS_HEADER} ${ULP_VAR_MAP_HEADER} ${ULP_ALIASES_LD}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${ULP_ARCH_FOLDER}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${ULP_BOARD_SCRIPTS_DIR}
      COMMAND ${CMAKE_COMMAND} -E touch ${ULP_VARS_HEADER}
      COMMAND ${CMAKE_COMMAND} -E touch ${ULP_ALIASES_LD}
      COMMAND bash -c "${_ESP_ULP_BASH_WRITE_VAR_MAP_INIT}"
      COMMENT "Creating shared ULP stub files"
      VERBATIM)

    add_custom_target(esp_ulp_shared_stubs DEPENDS ${ULP_VARS_HEADER})
    add_dependencies(arch esp_ulp_shared_stubs)
  endif()

  # ############################################################################
  # Compiler / linker flags
  # ############################################################################

  set(ULP_COMPILE_OPTS -Os -ggdb -march=rv32imac_zicsr_zifencei -mdiv
                       -fdata-sections -ffunction-sections)
  set(ULP_ASM_OPTS -Os -ggdb -march=rv32imac_zicsr_zifencei -x
                   assembler-with-cpp -D__ASSEMBLER__)
  set(ULP_LINK_OPTS
      -march=rv32imac_zicsr_zifencei --specs=nano.specs --specs=nosys.specs
      -nostartfiles -Wl,--no-warn-rwx-segments -Wl,--gc-sections)

  if(CONFIG_DEBUG_SYMBOLS)
    set(ULP_COMPILE_OPTS -O0 -ggdb -march=rv32imac_zicsr_zifencei -mdiv
                         -fdata-sections -ffunction-sections)
    set(ULP_ASM_OPTS -O0 -ggdb -march=rv32imac_zicsr_zifencei -x
                     assembler-with-cpp -D__ASSEMBLER__)
    if(CONFIG_ESPRESSIF_ULP_ENABLE_UBSAN)
      list(APPEND ULP_COMPILE_OPTS -fno-sanitize=shift-base
           -fsanitize=undefined)
      list(APPEND ULP_ASM_OPTS -fno-sanitize=shift-base -fsanitize=undefined)
    endif()
  endif()

  foreach(_ulp_inc ${ULP_INCLUDE_DIRS})
    list(APPEND ULP_INCLUDE_FLAGS -I${_ulp_inc})
  endforeach()

  # ############################################################################
  # Build
  # ############################################################################

  if(DEFINED ULP_APP_BIN AND ULP_APP_BIN MATCHES "\\.bin$")
    set(ULP_BIN_FILE_PATH ${ULP_APP_BIN})

    if(NOT EXISTS "${ULP_BIN_FILE_PATH}")
      message(FATAL_ERROR "ULP prebuilt binary not found: ${ULP_BIN_FILE_PATH}")
    endif()

    add_custom_command(
      OUTPUT ${ULP_CODE_HEADER}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${ULP_FOLDER}
      COMMAND ${XXD_PROGRAM} -i ${ULP_BIN_FILE_PATH} ${ULP_CODE_HEADER}
      COMMAND
        sed -i
        "s/unsigned char[^[]*\\[[^]]*\\]/unsigned char ${ULP_APP_NAME}_bin[]/g"
        ${ULP_CODE_HEADER}
      COMMAND
        sed -i "s/unsigned int[^=]* =/unsigned int ${ULP_APP_NAME}_bin_len =/g"
        ${ULP_CODE_HEADER}
      DEPENDS ${ULP_BIN_FILE_PATH} ${ULP_VAR_MAP_HEADER}
      COMMENT "Converting ULP prebuilt binary into header file"
      VERBATIM)
  else()
    if(NOT ULP_APP_C_SRCS AND NOT ULP_APP_ASM_SRCS)
      message(
        FATAL_ERROR
          "ULP source build requires ULP_APP_C_SRCS and/or ULP_APP_ASM_SRCS")
    endif()

    set(ULP_BIN_FILE ${ULP_FOLDER}/ulp.bin)
    set(ULP_BIN_FILE_PATH ${ULP_BIN_FILE})
    set(ULP_ELF_FILE ${ULP_FOLDER}/ulp.elf)
    set(ULP_MAP_FILE ${ULP_FOLDER}/ulp.map)
    set(ULP_SYM_FILE ${ULP_FOLDER}/ulp.sym)
    set(ULP_NUTTX_CONFIG_COPY ${ULP_FOLDER}/nuttx/config.h)

    foreach(_ulp_csrc ${ULP_APP_C_SRCS})
      list(APPEND ULP_APP_C_SOURCES ${ULP_APP_FOLDER}/${_ulp_csrc})
    endforeach()

    foreach(_ulp_asrc ${ULP_APP_ASM_SRCS})
      list(APPEND ULP_APP_ASM_SOURCES ${ULP_APP_FOLDER}/${_ulp_asrc})
    endforeach()

    if(ULP_APP_INCLUDES)
      foreach(_ulp_incl ${ULP_APP_INCLUDES})
        get_filename_component(_ulp_incl_dir ${_ulp_incl} DIRECTORY)
        list(APPEND ULP_APP_INCLUDE_DIRS ${_ulp_incl_dir})
      endforeach()
      list(REMOVE_DUPLICATES ULP_APP_INCLUDE_DIRS)
      list(APPEND ULP_INCLUDE_DIRS ${ULP_APP_INCLUDE_DIRS})
      set(ULP_INCLUDE_FLAGS)
      foreach(_ulp_inc ${ULP_INCLUDE_DIRS})
        list(APPEND ULP_INCLUDE_FLAGS -I${_ulp_inc})
      endforeach()
    endif()

    list(APPEND ULP_INCLUDE_DIRS ${ULP_FOLDER})
    list(APPEND ULP_INCLUDE_FLAGS -I${ULP_FOLDER})
    list(APPEND ULP_CSOURCES ${ULP_APP_C_SOURCES})
    list(APPEND ULP_ASOURCES ${ULP_APP_ASM_SOURCES})

    if(NOT EXISTS ${ULP_LPCORE_SECTIONS_LD})
      message(
        FATAL_ERROR "ULP linker template not found: ${ULP_LPCORE_SECTIONS_LD}")
    endif()

    add_custom_command(
      OUTPUT ${ULP_NUTTX_CONFIG_COPY}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${ULP_FOLDER}/nuttx
      COMMAND
        ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/include/nuttx/config.h
        ${ULP_NUTTX_CONFIG_COPY}
      DEPENDS nuttx_context ${ULP_VAR_MAP_HEADER}
      COMMENT "Copying nuttx/config.h for ULP build"
      VERBATIM)

    add_custom_command(
      OUTPUT ${ULP_SECTIONS_LD}
      COMMAND ${CMAKE_C_COMPILER} ${ULP_INCLUDE_FLAGS} -E -P -xc -o
              ${ULP_SECTIONS_LD} ${ULP_LPCORE_SECTIONS_LD}
      DEPENDS ${ULP_NUTTX_CONFIG_COPY} ${ULP_LPCORE_SECTIONS_LD}
      COMMENT "Preprocessing ULP linker sections"
      VERBATIM)

    add_custom_target(${ULP_PREPARE_TARGET} DEPENDS ${ULP_NUTTX_CONFIG_COPY}
                                                    ${ULP_SECTIONS_LD})

    add_library(${ULP_FIRMWARE_TARGET} OBJECT ${ULP_CSOURCES} ${ULP_ASOURCES})
    add_dependencies(${ULP_FIRMWARE_TARGET} ${ULP_PREPARE_TARGET})
    if(TARGET copy_esp3rdparty_headers)
      add_dependencies(${ULP_FIRMWARE_TARGET} copy_esp3rdparty_headers)
    endif()

    set_source_files_properties(
      ${ULP_CSOURCES} ${ULP_ASOURCES} PROPERTIES OBJECT_DEPENDS
                                                 "${ULP_NUTTX_CONFIG_COPY}")

    target_include_directories(${ULP_FIRMWARE_TARGET}
                               PRIVATE ${ULP_INCLUDE_DIRS})
    target_compile_definitions(${ULP_FIRMWARE_TARGET} PRIVATE IS_ULP_COCPU)
    target_compile_options(
      ${ULP_FIRMWARE_TARGET}
      PRIVATE ${ULP_COMPILE_OPTS}
              -include
              ${HAL}/nuttx/${CHIP_SERIES}/include/sdkconfig.h
              -Wno-shadow
              -Wno-undef
              -Wno-unused-variable
              -Wno-strict-prototypes
              -Wno-deprecated-declarations
              $<$<COMPILE_LANGUAGE:ASM>:${ULP_ASM_OPTS}>)

    add_custom_command(
      OUTPUT ${ULP_ELF_FILE}
      COMMAND
        ${CMAKE_C_COMPILER} ${ULP_LINK_OPTS} ${ULP_LDSCRIPTS} -Xlinker
        -Map=${ULP_MAP_FILE} "$<TARGET_OBJECTS:${ULP_FIRMWARE_TARGET}>" -o
        ${ULP_ELF_FILE}
      COMMAND_EXPAND_LISTS
      DEPENDS ${ULP_FIRMWARE_TARGET} ${ULP_SECTIONS_LD}
      COMMENT "Linking ULP firmware"
      VERBATIM)

    set(_ulp_postprocess_aliases
        [=[grep -E '^[[:space:]]*[a-zA-Z_][a-zA-Z0-9_]*[[:space:]]*=[[:space:]]*[0x]*[0-9a-fA-F]+;' "@ULP_MAIN_LD@" | while IFS= read -r line; do var_name=$(echo "$line" | sed -E 's/^[[:space:]]*([a-zA-Z_][a-zA-Z0-9_]*).*/\1/'); existing_line=$(grep -E "^[[:space:]]*${var_name}[[:space:]]*=" "@ULP_ALIASES_LD@" 2>/dev/null || true); if [ -n "$existing_line" ]; then if [ "$existing_line" != "$line" ]; then sed -i "/${existing_line}/c\\${line}" "@ULP_ALIASES_LD@"; fi; else echo "$line" >> "@ULP_ALIASES_LD@"; fi; done]=]
    )
    string(REPLACE "@ULP_MAIN_LD@" "${ULP_MAIN_LD}" _ulp_postprocess_aliases
                   "${_ulp_postprocess_aliases}")
    string(REPLACE "@ULP_ALIASES_LD@" "${ULP_ALIASES_LD}"
                   _ulp_postprocess_aliases "${_ulp_postprocess_aliases}")

    set(_ulp_postprocess_var_map
        [=[if ! grep -q "struct ulp_var_map_s ulp_var_map" "@ULP_VAR_MAP_HEADER@"; then printf '%s\n' '#include "nuttx/symtab.h"' '#include "ulp/ulp_vars.h"' '' 'struct ulp_var_map_s' '{' '  struct symtab_s sym;' '  size_t size;' '};' '' 'struct ulp_var_map_s ulp_var_map[] =' '{ };' > "@ULP_VAR_MAP_HEADER@"; fi; sed -i "/@ULP_PREFIX@/d" "@ULP_VAR_MAP_HEADER@"; flock -x "@ULP_LOCKFILE@" -c 'grep "@ULP_PREFIX@" "@ULP_MAIN_HEADER@" | while IFS= read -r line; do var=$(echo "$line" | grep -oP "@ULP_PREFIX@\w+(?=[;\[])" ); if [ -n "$var" ]; then size=$(echo "$line" | grep -oP "\[\d+\]" | grep -oP "\d+"); if [ -n "$size" ]; then size=$(( size * 4 )); else size=4; fi; sed -i "s/ };//" "@ULP_VAR_MAP_HEADER@"; echo -ne "  { .sym.sym_name = \"${var}\", .sym.sym_value = &${var}, .size = ${size}},\n };" >> "@ULP_VAR_MAP_HEADER@"; fi; done']=]
    )
    string(REPLACE "@ULP_VAR_MAP_HEADER@" "${ULP_VAR_MAP_HEADER}"
                   _ulp_postprocess_var_map "${_ulp_postprocess_var_map}")
    string(REPLACE "@ULP_LOCKFILE@" "${ULP_LOCKFILE}" _ulp_postprocess_var_map
                   "${_ulp_postprocess_var_map}")
    string(REPLACE "@ULP_MAIN_HEADER@" "${ULP_MAIN_HEADER}"
                   _ulp_postprocess_var_map "${_ulp_postprocess_var_map}")
    string(REPLACE "@ULP_PREFIX@" "${ULP_PREFIX}" _ulp_postprocess_var_map
                   "${_ulp_postprocess_var_map}")

    add_custom_command(
      OUTPUT ${ULP_CODE_HEADER} ${ULP_BIN_FILE} ${ULP_SYM_FILE}
             ${ULP_MAIN_HEADER} ${ULP_MAIN_LD}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${ULP_FOLDER}
      COMMAND ${CMAKE_OBJCOPY} -O binary ${ULP_ELF_FILE} ${ULP_BIN_FILE}
      COMMAND ${ULP_READELF} -sW ${ULP_ELF_FILE} > ${ULP_SYM_FILE}
      COMMAND ${Python3_EXECUTABLE} ${ULP_MAPGEN_TOOL} -s ${ULP_SYM_FILE} -o
              ${ULP_FOLDER}/ulp_main --base ${ULP_BASE} --prefix ${ULP_PREFIX}
      COMMAND bash -c "${_ulp_postprocess_aliases}"
      COMMAND sed -i "/${ULP_PREFIX}/d" ${ULP_VARS_HEADER}
      COMMAND
        bash -c
        "grep \"extern uint32_t\" \"${ULP_MAIN_HEADER}\" >> \"${ULP_VARS_HEADER}\""
      COMMAND bash -c "${_ulp_postprocess_var_map}"
      COMMAND ${XXD_PROGRAM} -i ${ULP_BIN_FILE_PATH} ${ULP_CODE_HEADER}
      COMMAND
        sed -i
        "s/unsigned char[^[]*\\[[^]]*\\]/unsigned char ${ULP_APP_NAME}_bin[]/g"
        ${ULP_CODE_HEADER}
      COMMAND
        sed -i "s/unsigned int[^=]* =/unsigned int ${ULP_APP_NAME}_bin_len =/g"
        ${ULP_CODE_HEADER}
      DEPENDS ${ULP_ELF_FILE} ${ULP_VAR_MAP_HEADER}
      COMMENT "Generating ULP binary, symbol map, and headers"
      VERBATIM)
  endif()

  add_custom_target(${ULP_BIN_TARGET} DEPENDS ${ULP_CODE_HEADER})
  if(TARGET ${ULP_PREPARE_TARGET})
    add_dependencies(${ULP_BIN_TARGET} ${ULP_PREPARE_TARGET})
  endif()
  add_dependencies(board ${ULP_BIN_TARGET})
  add_dependencies(arch ${ULP_BIN_TARGET})

  set_property(GLOBAL APPEND PROPERTY ESP_ULP_BIN_TARGETS ${ULP_BIN_TARGET})

  function(_esp_ulp_add_nuttx_link_deps)
    get_property(_esp_ulp_bins GLOBAL PROPERTY ESP_ULP_BIN_TARGETS)
    if(NOT _esp_ulp_bins)
      return()
    endif()

    foreach(_esp_ulp_bin_target ${_esp_ulp_bins})
      if(TARGET ${_esp_ulp_bin_target})
        add_dependencies(nuttx ${_esp_ulp_bin_target})
        if(TARGET ldscript_tmp_ulp_aliases.ld)
          add_dependencies(ldscript_tmp_ulp_aliases.ld ${_esp_ulp_bin_target})
        endif()
      endif()
    endforeach()

    if(TARGET ldscript_tmp_ulp_aliases.ld AND TARGET esp_ulp_shared_stubs)
      add_dependencies(ldscript_tmp_ulp_aliases.ld esp_ulp_shared_stubs)
    endif()
  endfunction()

  get_property(_esp_ulp_defer GLOBAL PROPERTY ESP_ULP_LINK_DEFER_REGISTERED)
  if(NOT _esp_ulp_defer)
    set_property(GLOBAL PROPERTY ESP_ULP_LINK_DEFER_REGISTERED TRUE)
    cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL
                   _esp_ulp_add_nuttx_link_deps)
  endif()

  # ############################################################################
  # Clean hook
  # ############################################################################

  set_property(
    DIRECTORY ${CMAKE_SOURCE_DIR}
    APPEND
    PROPERTY ADDITIONAL_CLEAN_FILES ${ULP_FOLDER} ${ULP_ARCH_FOLDER}
             ${ULP_ALIASES_LD} ${ULP_ARCH_FOLDER}/.ulp_shared_stubs_stamp)

endif()
