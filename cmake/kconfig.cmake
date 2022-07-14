# SPDX-License-Identifier: Apache-2.0

# Folders needed for conf/mconf files (kconfig has no method of redirecting all output files).
# conf/mconf needs to be run from a different directory because of: GH-3408
set(PROJECT_BINARY_DIR ${CMAKE_BINARY_DIR})

set(SDK_BASE ${CMAKE_SOURCE_DIR})
set($ENV{SDK_BASE} ${SDK_BASE})
set(ARCH sim)
set(ARCH_DIR ${SDK_BASE}/arch/${ARCH})
set(SOC sim)
set(SOC_DIR ${SDK_BASE}/soc/${ARCH}/${SOC})
set(BOARD sim)
set(PRODUCT nsh)
set(BOARD_DIR ${SDK_BASE}/boards/${ARCH}/${SOC}/${BOARD}/configs/${PRODUCT})
set(APPLICATION_SOURCE_DIR ${CMAKE_SOURCE_DIR})
#set(CROSS_COMPILER $ENV{CROSS_COMPILER})
#set(KERNEL $ENV{KERNEL})
message(STATUS "SDK_BASE: ${SDK_BASE}")

function(import_kconfig prefix kconfig_fragment)
  # Parse the lines prefixed with 'prefix' in ${kconfig_fragment}
  file(
    STRINGS
    ${kconfig_fragment}
    DOT_CONFIG_LIST
    REGEX "^${prefix}"
    ENCODING "UTF-8"
  )

  foreach (CONFIG ${DOT_CONFIG_LIST})
    # CONFIG could look like: CONFIG_NET_BUF=y

    # Match the first part, the variable name
    string(REGEX MATCH "[^=]+" CONF_VARIABLE_NAME ${CONFIG})

    # Match the second part, variable value
    string(REGEX MATCH "=(.+$)" CONF_VARIABLE_VALUE ${CONFIG})
    # The variable name match we just did included the '=' symbol. To just get the
    # part on the RHS we use match group 1
    set(CONF_VARIABLE_VALUE ${CMAKE_MATCH_1})

    if("${CONF_VARIABLE_VALUE}" MATCHES "^\"(.*)\"$") # Is surrounded by quotes
      set(CONF_VARIABLE_VALUE ${CMAKE_MATCH_1})
    endif()

    set("${CONF_VARIABLE_NAME}" "${CONF_VARIABLE_VALUE}" PARENT_SCOPE)
  endforeach()
endfunction()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/kconfig/include/generated)
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/kconfig/include/config)

if(KCONFIG_ROOT)
  # KCONFIG_ROOT has either been specified as a CMake variable or is
  # already in the CMakeCache.txt. This has precedence.
elseif(EXISTS   ${APPLICATION_SOURCE_DIR}/Kconfig)
  set(KCONFIG_ROOT ${APPLICATION_SOURCE_DIR}/Kconfig)
else()
  set(KCONFIG_ROOT ${SDK_BASE}/Kconfig)
endif()

set(BOARD_DEFCONFIG ${BOARD_DIR}/defconfig)
set(DOTCONFIG                  ${PROJECT_BINARY_DIR}/.config)
set(PARSED_KCONFIG_SOURCES_TXT ${PROJECT_BINARY_DIR}/kconfig/sources.txt)
set(AUTOCONF_H ${PROJECT_BINARY_DIR}/config.h)
set(GENERATED_DTS_BOARD_CONF .config)
set(SCRIPTS ${SDK_BASE}/tools)

if(CONF_FILE)
string(REPLACE " " ";" CONF_FILE_AS_LIST "${CONF_FILE}")
endif()

if(OVERLAY_CONFIG)
  string(REPLACE " " ";" OVERLAY_CONFIG_AS_LIST "${OVERLAY_CONFIG}")
endif()

set(ENV{srctree}            ${SDK_BASE})
set(ENV{KERNELVERSION}      ${KERNELVERSION})
set(ENV{KCONFIG_CONFIG}     ${DOTCONFIG})
set(ENV{PYTHON_EXECUTABLE} ${PYTHON_EXECUTABLE})

# Set environment variables so that Kconfig can prune Kconfig source
# files for other architectures
set(ENV{ARCH}      ${ARCH})
set(ENV{BOARD_DIR} ${BOARD_DIR})
set(ENV{SOC_DIR}   ${SOC_DIR})
set(ENV{CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR})
set(ENV{ARCH_DIR}   ${ARCH_DIR})
set(ENV{GENERATED_DTS_BOARD_CONF} ${GENERATED_DTS_BOARD_CONF})

# Allow out-of-tree users to add their own Kconfig python frontend
# targets by appending targets to the CMake list
# 'EXTRA_KCONFIG_TARGETS' and setting variables named
# 'EXTRA_KCONFIG_TARGET_COMMAND_FOR_<target>'
#
# e.g.
# cmake -DEXTRA_KCONFIG_TARGETS=cli
# -DEXTRA_KCONFIG_TARGET_COMMAND_FOR_cli=cli_kconfig_frontend.py

set(EXTRA_KCONFIG_TARGET_COMMAND_FOR_menuconfig
  ${SCRIPTS}/kconfig/menuconfig.py
  )

foreach(kconfig_target
    menuconfig
    guiconfig
    hardenconfig
    ${EXTRA_KCONFIG_TARGETS}
    )
  add_custom_target(
    ${kconfig_target}
    ${CMAKE_COMMAND} -E env
    PYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
    srctree=${SDK_BASE}
    KERNELVERSION=${KERNELVERSION}
    KCONFIG_CONFIG=${DOTCONFIG}
    ARCH=$ENV{ARCH}
    BOARD_DIR=$ENV{BOARD_DIR}
    SOC_DIR=$ENV{SOC_DIR}
    SDK_BASE=$ENV{SDK_BASE}
    CMAKE_BINARY_DIR=$ENV{CMAKE_BINARY_DIR}
    ARCH_DIR=$ENV{ARCH_DIR}
    GENERATED_DTS_BOARD_CONF=${GENERATED_DTS_BOARD_CONF}
    ${PYTHON_EXECUTABLE}
    ${EXTRA_KCONFIG_TARGET_COMMAND_FOR_${kconfig_target}}
    ${KCONFIG_ROOT}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/kconfig
    USES_TERMINAL
    )
endforeach()

# Support assigning Kconfig symbols on the command-line with CMake
# cache variables prefixed with 'CONFIG_'. This feature is
# experimental and undocumented until it has undergone more
# user-testing.
unset(EXTRA_KCONFIG_OPTIONS)
get_cmake_property(cache_variable_names CACHE_VARIABLES)
foreach (name ${cache_variable_names})
  if("${name}" MATCHES "^CONFIG_")
    # When a cache variable starts with 'CONFIG_', it is assumed to be
    # a CLI Kconfig symbol assignment.
    set(EXTRA_KCONFIG_OPTIONS
      "${EXTRA_KCONFIG_OPTIONS}\n${name}=${${name}}"
      )
  endif()
endforeach()

if(EXTRA_KCONFIG_OPTIONS)
  set(EXTRA_KCONFIG_OPTIONS_FILE ${PROJECT_BINARY_DIR}/misc/generated/extra_kconfig_options.conf)
  file(WRITE
    ${EXTRA_KCONFIG_OPTIONS_FILE}
    ${EXTRA_KCONFIG_OPTIONS}
    )
endif()

# Bring in extra configuration files dropped in by the user or anyone else;
# make sure they are set at the end so we can override any other setting
file(GLOB config_files ${APPLICATION_BINARY_DIR}/*.conf)
list(SORT config_files)
set(
  merge_config_files
  ${BOARD_DEFCONFIG}
  ${CONF_FILE_AS_LIST}
  ${OVERLAY_CONFIG_AS_LIST}
  ${EXTRA_KCONFIG_OPTIONS_FILE}
  ${config_files}
)

# Create a list of absolute paths to the .config sources from
# merge_config_files, which is a mix of absolute and relative paths.
set(merge_config_files_with_absolute_paths "")
foreach(f ${merge_config_files})
  if(IS_ABSOLUTE ${f})
    set(path ${f})
  else()
    set(path ${APPLICATION_SOURCE_DIR}/${f})
  endif()

  list(APPEND merge_config_files_with_absolute_paths ${path})
endforeach()

foreach(f ${merge_config_files_with_absolute_paths})
  if(NOT EXISTS ${f} OR IS_DIRECTORY ${f})
    message(FATAL_ERROR "File not found: ${f}")
  endif()
endforeach()

# Calculate a checksum of merge_config_files to determine if we need
# to re-generate .config
set(merge_config_files_checksum "")
foreach(f ${merge_config_files_with_absolute_paths})
  file(MD5 ${f} checksum)
  set(merge_config_files_checksum "${merge_config_files_checksum}${checksum}")
endforeach()

# Create a new .config if it does not exists, or if the checksum of
# the dependencies has changed
set(merge_config_files_checksum_file ${PROJECT_BINARY_DIR}/.cmake.dotconfig.checksum)
set(CREATE_NEW_DOTCONFIG 1)
# Check if the checksum file exists too before trying to open it, though it
# should under normal circumstances
if(EXISTS ${DOTCONFIG} AND EXISTS ${merge_config_files_checksum_file})
  # Read out what the checksum was previously
  file(READ
    ${merge_config_files_checksum_file}
    merge_config_files_checksum_prev
    )
  if(
      ${merge_config_files_checksum} STREQUAL
      ${merge_config_files_checksum_prev}
      )
    # Checksum is the same as before
    set(CREATE_NEW_DOTCONFIG 0)
  endif()
endif()

if(CREATE_NEW_DOTCONFIG)
  file(WRITE
    ${merge_config_files_checksum_file}
    ${merge_config_files_checksum}
    )

  set(merge_fragments ${merge_config_files})
else()
  set(merge_fragments ${DOTCONFIG})
endif()

message(STATUS KCONFIG_ROOT: ${KCONFIG_ROOT})
message(STATUS DOTCONFIG: ${DOTCONFIG})
message(STATUS AUTOCONF_H: ${AUTOCONF_H})
message(STATUS PARSED_KCONFIG_SOURCES_TXT: ${PARSED_KCONFIG_SOURCES_TXT})
message(STATUS merge_fragments: ${merge_fragments})

include_directories(
    PUBLIC
	${PROJECT_BINARY_DIR}
	)

execute_process(
  COMMAND
${PYTHON_EXECUTABLE}
  ${SCRIPTS}/kconfig/kconfig.py
#  --zephyr-base ${SDK_BASE}
  ${KCONFIG_ROOT}
  ${DOTCONFIG}
  ${AUTOCONF_H}
  ${PARSED_KCONFIG_SOURCES_TXT}
  ${merge_fragments}
  WORKING_DIRECTORY ${APPLICATION_SOURCE_DIR}
  # The working directory is set to the app dir such that the user
  # can use relative paths in CONF_FILE, e.g. CONF_FILE=nrf5.conf
  RESULT_VARIABLE ret
  )
if(NOT "${ret}" STREQUAL "0")
  message(FATAL_ERROR "command failed with return code: ${ret}")
endif()

# Read out the list of 'Kconfig' sources that were used by the engine.
file(STRINGS ${PARSED_KCONFIG_SOURCES_TXT} PARSED_KCONFIG_SOURCES_LIST)

# Force CMAKE configure when the Kconfig sources or configuration files changes.
foreach(kconfig_input
    ${merge_config_files}
    ${DOTCONFIG}
    ${PARSED_KCONFIG_SOURCES_LIST}
    )
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${kconfig_input})
endforeach()

add_custom_target(config-sanitycheck DEPENDS ${DOTCONFIG})

# Remove the CLI Kconfig symbols from the namespace and
# CMakeCache.txt. If the symbols end up in DOTCONFIG they will be
# re-introduced to the namespace through 'import_kconfig'.
foreach (name ${cache_variable_names})
  if("${name}" MATCHES "^CONFIG_")
    unset(${name})
    unset(${name} CACHE)
  endif()
endforeach()

# Parse the lines prefixed with CONFIG_ in the .config file from Kconfig
import_kconfig(CONFIG_ ${DOTCONFIG})

# Re-introduce the CLI Kconfig symbols that survived
foreach (name ${cache_variable_names})
  if("${name}" MATCHES "^CONFIG_")
    if(DEFINED ${name})
      set(${name} ${${name}} CACHE STRING "")
    endif()
  endif()
endforeach()
