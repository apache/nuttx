# ##############################################################################
# boards/arm/imxrt/frdm-imxrt1186/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

target_sources(board PRIVATE imxrt_boot.c imxrt_bringup.c imxrt_clockconfig.c
                             imxrt_gpio.c)

if(CONFIG_FRDM_IMXRT1186_XIP)
  target_sources(
    board PRIVATE imxrt_flexspi_nor_boot.c imxrt_flexspi_nor_flash.c
                  imxrt_flexspi_nor_xmcd.c)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/xip.ld")
elseif(CONFIG_IMXRT_NETC)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/netc.ld")
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/dtcm.ld")
endif()
