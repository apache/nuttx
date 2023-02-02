# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(serial_nor)
sdk_inc(usb)

add_subdirectory(enet_phy)
add_subdirectory_ifndef(CONFIG_NDEBUG_CONSOLE debug_console)
add_subdirectory_ifdef(CONFIG_CAMERA camera)
add_subdirectory_ifdef(CONFIG_CODEC codec)
add_subdirectory_ifdef(CONFIG_USB_HOST usb/host)
add_subdirectory_ifdef(CONFIG_USB_DEVICE usb/device)
add_subdirectory_ifdef(CONFIG_TOUCH touch)
add_subdirectory_ifdef(CONFIG_HPM_ADC adc)
add_subdirectory_ifdef(CONFIG_HPM_SPI spi)
add_subdirectory_ifdef(CONFIG_DMA_MANAGER dma_manager)
add_subdirectory_ifdef(CONFIG_IPC_EVENT_MGR ipc_event_mgr)


