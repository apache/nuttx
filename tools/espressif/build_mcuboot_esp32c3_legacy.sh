#!/usr/bin/env bash

SCRIPT_NAME=$(basename "${BASH_SOURCE[0]}")

usage() {
  echo ""
  echo "USAGE: ${SCRIPT_NAME} [-h] -c <chip> -f <config> -p <path> -e <hal>"
  echo ""
  echo "Where:"
  echo "  -c <chip> Target chip"
  echo "  -f <config> Path to file containing configuration options"
  echo "  -p <path> Path to execute the script"
  echo "  -e <hal> Path to HAL directory"
  echo "  -h Show usage and terminate"
  echo ""
}

build_mcuboot() {
  local target=${1}
  local config=${2}
  local mcuboot_dir="${exec_path}/mcuboot"
  local build_dir=".build-${target}"
  local source_dir="boot/espressif"
  local output_dir="${exec_path}/out"
  local toolchain_file="tools/toolchain-${target}.cmake"
  local mcuboot_config
  local mcuboot_flashsize
  local mcuboot_flashmode
  local mcuboot_flashfreq
  local make_generator

  mcuboot_config=$(realpath "${config:-${exec_path}/mcuboot.conf}")

  # Try parsing Flash parameters from the mcuboot config file.
  # If not found, let's assume some commonplace values.

  mcuboot_flashsize=$(sed -n 's/^CONFIG_ESPTOOLPY_FLASHSIZE_\(.*\)MB=1/\1MB/p' "${mcuboot_config}")
  if [ -z "${mcuboot_flashsize}" ]; then
    mcuboot_flashsize="4MB"
  fi

  mcuboot_flashmode=$(sed -n 's/^CONFIG_ESPTOOLPY_FLASHMODE_\(.*\)=1/\L\1/p' "${mcuboot_config}")
  if [ -z "${mcuboot_flashmode}" ]; then
    mcuboot_flashmode="dio"
  fi

  mcuboot_flashfreq=$(sed -n 's/^CONFIG_ESPTOOLPY_FLASHFREQ_\(.*\)M=1/\1m/p' "${mcuboot_config}")
  if [ -z "${mcuboot_flashfreq}" ]; then
    mcuboot_flashfreq="40m"
  fi

  pushd "${exec_path}" &>/dev/null
  mkdir -p "${output_dir}" &>/dev/null

  # Build with Ninja if installed

  if command -v ninja &>/dev/null; then
    make_generator="-GNinja"
  fi

  # Build bootloader for selected target

  cd "${mcuboot_dir}" &>/dev/null
  cmake -DCMAKE_TOOLCHAIN_FILE="${toolchain_file}"     \
        -DMCUBOOT_TARGET="${target}"                   \
        -DMCUBOOT_CONFIG_FILE="${mcuboot_config}"      \
        -DESP_HAL_PATH="${esp_hal}"                    \
        -DCONFIG_ESP_FLASH_SIZE="${mcuboot_flashsize}" \
        -DESP_FLASH_MODE="${mcuboot_flashmode}"        \
        -DESP_FLASH_FREQ="${mcuboot_flashfreq}"        \
        -B "${build_dir}"                              \
        "${make_generator}"                            \
        "${source_dir}"
  cmake --build "${build_dir}"/

  # Copy bootloader binary file to output directory

  cp "${build_dir}"/mcuboot_"${target}".bin "${output_dir}"/mcuboot-"${target}".bin &>/dev/null

  # Remove build directory

  rm -rf "${build_dir}" &>/dev/null

  popd &>/dev/null
}

while getopts ":hc:f:p:e:" arg; do
  case "${arg}" in
    c)
      chip=${OPTARG}
      ;;
    f)
      config=${OPTARG}
      ;;
    p)
      exec_path=${OPTARG}
      ;;
    e)
      esp_hal=${OPTARG}
      ;;
    h)
      usage
      exit 0
      ;;
    *)
      usage
      exit 1
      ;;
  esac
done

if [ -z "${chip}" ]; then
  printf "ERROR: Missing target chip.\n"
  usage
  exit 1
fi

if ! [ -d "${esp_hal}" ]; then
  printf "ERROR: Invalid HAL path.\n"
  usage
  exit 1
fi

if [ ! -d "${exec_path}" ]; then
  printf "ERROR: Invalid exec path.\n"
  usage
  exit 1
fi

if [ -n "${config}" ] && [ ! -f "${config}" ]; then
  printf "ERROR: Configuration file %s not found.\n" "${config}"
  usage
  exit 1
fi

build_mcuboot "${chip}" "${config}"
