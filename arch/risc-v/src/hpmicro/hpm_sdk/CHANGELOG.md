# Change Log

## [1.00.0] - 2022-12-31:

Main changes since 0.14.0

Tested Segger Embedded Studio Version: 7.10

### Changed:
  - boards: add version info in banner.
  - board: lcdc: move panel para to board
  - components: spi component to support to transfer different width of data in dma handshake mode
  - drivers: rename dram to femc
  - middleware: cherryusb update to v0.7.0
  - middleware: erpc: update for support rtos
  - middleware: hpm_mcl: update api naming
  - samples: erpc: rename erpc_matrix_multiply_rpmsg to erpc_matrix_multiply_rpmsg_bm
  - samples: erpc: reorganize erpc_matrix_multiply_rpmsg samples
  - samples: erpc: move samples folder to erpc_matrix_multiply_rpmsg
  - samples: update i2c/spi dma channel and dmamux channel definition
  - ses: project template: use demo.* as output file naming.
  - soc: HPM6750: toolchains: update linker files
  - soc: HPM6360: toolchains: update linker files

### Added:
  - arch: riscv: add read_clear_csr() API
  - component: add wm8960 support
  - component: add usb device iso transfer support
  - drivers: hpm_common: include hpm_sdk_version.h.
  - drivers: common: add NOP and WFI.
  - drivers: uart: add api to recv/send byte directly.
  - soc: HPM6750 linker files: rename rpmsg_sh_mem to sh_mem
  - cmake: add sdk version header file generation.
  - middleware: tinyusb: add audio class
  - middleware: hpm_math: add NN library
  - samples: erpc: add erpc_two_way_rpc_rpmsg_rtos sample
  - samples: erpc: add erpc_matrix_multiply_rpmsg_rtos sample
  - samples: add power mode switch demo. #282
  - samples: drivers: adc: add a temperature measurement demo
  - samples: drviers: femc: add sram sample
  - samples: lwip demo for FreeRTOS
  - samples: lwip demo of interrupt usage
  - samples: tinyusb: add uac2 demo
  - samples: tflm: add face detection demo
  - samples: tflm: add MLPerf Tiny benchmark demo
  - samples: lwip: common: feature: add a LPI interrupt process

### Fixed:
  - drivers:interrupt: Fix FPU context crashing in nested irq case
  - drivers: src: adc: fix adc result in period mode
  - segger: update app directory structure in SES.
  - soc: disable irq during cache maintenance
  - middleware: lvgl: fps calculation to No.
  - samples: get off level based on board api.
  - samples: multicore: hello: core1 rgb led does not change correctly.

## [0.14.0] - 2022-10-31:

Main changes since 0.13.0

Tested Segger Embedded Studio Version: 6.34a

### Changed:
  - drivers: inc: update adc driver
  - drivers: gptmr: update reload value
  - components: enet_phy: optimize APIs
  - components: enet_phy: dp83867: rename functions
  - components: conditionally add debug_console
  - middleware: add cherryusb (0.6.0)
  - middleware: littlevgl: update to v8.3.1
  - middleware: fatfs: file name encoding in utf-8
  - middleware: freertos: support nested irq handling
  - middleware: hpm_mcl: Optimized motor control foc speed
  - samples: drivers: dma src move to dma_general_transfer folder
  - samples: lwip: update all static addresses and all gateway addresses
  - samples: lwip: lwip_iperf: optimize the interactive log
  - samples: tinyusb: device: hid_generic_inout: optimize the python script
  - openocd: HPM6750A1 silicon in hpm6750-dual-core.cfg
  - header file: update the enet/conctl register files
  - header file: Update TRGMUX0 pin input source definition
  - header file: update the ADC12 header files
  - scripts: ses: organize file in ses according to real path.
  - cmake: split gcc and ses source
  - board: bump HPM6750 DCDC voltage to 1200mv

### Fixed:
  - drivers: pdma: fix scale api issue
  - drivers: i2c: update DATACNT processing
  - drivers: i2s: fix i2s interrupt workaround in i2s_init
  - drivers: pwm: fix pwm capture function error
  - drivers: pwm: fix pwm capture configuration error
  - drivers: clock: fix error in clock_set_xxx_source
  - drivers: wdg: timeout calculation error
  - drivers: trgm_drv: bugfix: include error
  - drivers: romapi: call fencei after flash erase/write operation
  - drivers: watchdog: overflow
  - drivers: usb: host controller initialization issue
  - drivers: i2s: i2s_enable() issue fix
  - middleware: tinyusb: src: class: fix the HID report desc macro
  - samples: drivers: adc: optimize all ADC demos
  - samples: drivers: adc: fix all channel initializations  without a default value
  - samples: motor_ctrl: hardware trigger api usage error
  - samples: audio_codec: update clock process for 44100 sample rate
  - samples: lcdc: boundary pixel is incorrenct
  - samples: multicore: BOOT_HEADER was missing in multicore core0 example
  - samples: drivers: i2s: correct audio data if depth < 32bit.
  - samples: hpm_math: fft_perf_test: Fix error printing information
  - samples: fix pdm2dao noise problem
  - samples: lwip: fix the enet throughput degradation
  - soc: correct address overlapping SES XIP linker file
  - soc: correct the interrupt context switch issue
  - soc: fix Lack of interrupt claim for swi
  - boards: fix some rmii reference clock APIs

### Added:
  - boards: add hpm6750evk2 support
  - soc: add initfini.c
  - drivers: lcdc: add y8 support
  - drivers: spi: update data_length processing
  - drivers: spi:  add api to enable/disable spi dma request
  - drivers: pmp: Add pmp_config_entry API
  - components: add ipc_event_mgr
  - component: spi: add cache maintain
  - middleware: add erpc
  - samples: lwip: support self-adaptive port speed and duplex mode
  - samples: jpeg: support grayscale
  - samples: drivers: spi: use api to get data length
  - samples: drivers: spi:  add interrupt b2b sample
  - samples: drivers: mbx: add singlecore samples
  - samples: drivers: add dma circle transfer
  - samples: drivers: pwm: add pwm capture demo
  - samples: provide OTP API demo
  - samples: add cherryusb hid/msc/cdc samples
  - samples: add Guomi API example
  - samples: add erpc sample
  - samples: add segger_rtt

## [0.13.0] - 2022-07-31:

Main changes since 0.12.1

### Changed:
  - drivers: adc: fix sequence and preemption mode
  - drivers: usb: automatically change transceiver type in host mode
  - drivers: pwm: modify the interface for setting the pwm shadow register trigger function
  - drivers: sdxc: driver improvement
  - samples: jpeg demo update
  - samples: motor_ctrl: Optimize motor control performance
  - samples: motor_ctrl: Modify the pwm update method to be hardware triggered
  - samples: audio_codec: wav: Modify the interface
  - samples: audio_codec: add dao support
  - samples: tinyusb: update all USB project names
  - samples: adc: place DMA buffer in noncacheable
  - components: usb: update two struct definitions for the hcd_data_t and the dcd_data_t
  - middleware: littlevgl: update double buffer refresh approach.
  - middleware: fatfs: enable exfat format
  - middleware: fatfs: change 'USB' to lowercase
  - middleware: hpm_math: remove cache operations from the ffa interface
  - scripts: ses: update file path using $(HPM_SDK_BASE)

### Fixed:
  - drivers: gpiom: fix spelling errors in API names
  - drivers: enet: fix ptp time offset update
  - drivers: cam: fix store mode configuration
  - drivers: can: Fix the issue can timestamp cannot be enabled
  - drivers: gpio: incorrect return type gpio_get_port_interrupt_flags.
  - middleware: audio_codec: decoder_wav: fix wav codec problem
  - middleware: tinyusb: fix the alignment of _hcd_data
  - soc: driver: clock: i2s get clock error
  - soc: hpm_misc.h: fix incorrect system address mapping.
  - soc: SVD files: fix SDP peripheral reset value problem
  - soc: disable vector mode explicitly.
  - boards: correct device name in ses project
  - board: correct the printf format for frequency and register base
  - python: fix ses project generation on windows
  - samples: adc: fix not working in sequence mode and preemption mode
  - ses: set obj path for common configuration.
  - cmake: ses: put app source into separate category in ses.

### Added:
  - components: enet_phy: add rtl8211 driver
  - components: hpm_spi: add initial SPI component
  - components: add DMA Manager
  - drivers: cam: add new API
  - drivers: uart: add uart_set_baudrate API
  - drivers: spi: update dma transfer API
  - samples: tinyusb: host: add a hid demo
  - samples: drivers: spi: add master_trans_large_amount_of_data example
  - samples: decoder_wav: Add support for different bit rates and bits
  - samples: drivers: audio: automatic acquisition of i2s clock frequency
  - middleware: tinyusb: update portable file for USB host

## [0.12.1] - 2022-07-01:

Main changes since 0.12.0

### Fixed:
  - update lwipopt.h for updated enet driver

## [0.12.0] - 2022-06-30:

Tested with SES 6.32

Main changes since 0.11.0

### Changed:
  - boards: hpm6360evk has been renamed to hpm6300evk
  - boards: Use CSR_CYCLE in the clock_delay function
  - soc: hpm_soc.h: include hpm_common.h
  - driver: rename hpm_pmu_drv.c to hpm_pcfg_drv.h
  - driver: spi: change to non-blocking interfaces
  - driver: dma: update driver to adapt to different DMA instance constraints
  - components: enet_phy: update dp83867 driver
  - components: enet_phy: remove rtl8211 driver
  - cmake: enable nano newlib by default
  - cmake: move distclean to the beginning
  - middleware: littlevgl enable PDMA by default
  - middleware: tinyusb: upgrade to 0.13.0
  - samples: tinyusb: device: cdc_msc: adjust buffer size
  - samples: drivers: plic: use gpio toggle count as nested irq

### Fixed:
  - drivers: pwm: fix function name inconsistency bug
  - drivers: enet: remove "board.h" in enet driver
  - drivers: clock: Fix bugs in hpm6360 clock driver
  - drivers: clock: returns wrong adc/dac clock in HPM6360
  - drivers: dram: failed to configure 8bit mode.
  - freertos: fix issue about running on core1
  - boards: hpm6300evk pmp_entry set initial value
  - middleware: littlevgl: fix pdma cache op issue
  - middleware: littlevgl: fix doxygen markdown for pdma driver
  - middleware: lvgl: ses: update ram linker.
  - middleware: hpm_math: fix ffa cache size error
  - samples: audio_codec: remove the wrong dependency
  - samples: FATFS demo cannot support write/read if FATFS passes unaligned buffer address

### Added:
  - samples: add memstress and flash stress
  - drivers: clock: add implement common delay function based on mcycle and mcycleh register
  - boards: add hpm6300evk support
  - boards: hpm6750evkmini: motor control support
  - soc: add svd files
  - middleware: lwip: iperf: enable iperf and add udp function
  - samples: lwip: add lwip_iperf

## [0.11.0] - 2022-05-31:
Main changes since 0.10.0-hpm6360-er

### Changed:
  - readme: change the location of the starting document
  - components: change hal_adc_xx to hpm_adc_xx
  - drivers: update the I2C driver
  - cmake: change default rv_arch to rv32imac
  - cmake: modified in a zephyr-compatible way
  - drivers: uart: redefined struct with zephyr
  - cmake: change soc and board name
  - cmake: ses: support enable ext_dsp in project file.
  - samples/tinyusb/host: optimize the toggle rate of the blinking led
  - samples/tinyusb/device: replace mchtimer with board_timer for led_linking_taskk
  - samples/tinyusb/device: optimize the implement of led_blinking_task
  - soc: update IP header and soc header files

### Fixed:
  - samples: motor_ctrl: bldc_block: fix some error
  - middleware: hpm_math: fix andes toolchain compile error
  - middleware: hpm_math: fix libdsp.a error
  - middleware: hpm_math: add ext-dsp for SES
  - samples: openocd_algo: fix func_table placement.
  - boards: hpm6360evk: correct cpu frequency
  - boards: fix warning caused by code irregularities
  - boards: LED status is not the same between hpm6750mini rev-A and rev-B
  - boards: update board_led_write.

## Added:
  - driver: add spi_setup_dma_transfer() API
  - middleware: hpm_math: add software fft function
  - middleware: hpm_math: add ffa to hpm_math
  - samples: i2c: update the sample b2b
  - samples: hpm_math: add fft_performance demo
  - samples: add SPI DMA sample
  - samples: multicore coremark using debug console only
  - samples: 1588: add ptp v1 master/slave

## [0.10.0_hpm6300] - 2022-05-16:
Main changes since 0.10.0

### Changed:
  - ip register header file update
  - ses_proj: using generated complete cpu_regs_xml to replace general cpu registers xml
  - drivers: gpiom: move gpiom_gpio_t into hpm_gpiom_src.h
  - drivers: update the enet driver
  - samples: dram: change sdram test address in dma demo
  - samples: drivers: mbx: exclude flash targets for core0 as well.
  - samples: lwip: update ethernetif.c

### Fixed:
  - drivers: ptpc: update ptpc_init_timer_with_initial.
  - drivers: can: bug fix and update
  - soc: correct vector table inclusion and swi name
  - env.sh: fix HPM_SDK_BASE setting problem with msys.
  - samples: sha256_example: failed to run case 13 and 14 with gcc.

### Added:
  - soc: add HPM6360 support
  - drivers: add ffa, pllctlv2, dac
  - boards: add hpm6360evk support
  - components: add adcx module
  - components/enet_phy: add the RTL8201 driver
  - samples: drivers: dac, ffa
  - samples: rgb_red: add evkmini ver B support

## [0.10.0] - 2022-04-30:
Main changes since 0.9.0

### Changed:
  - drivers: gptmr: split irq enable/disable interface
  - drivers: can: 155 Update SJW setting in CAN driver
  - soc: HPM6750: rename safe stack symbol in link script
  - components: debug_console: wait uart tx flag on writing.

### Added:
  - drivers: enet: add 1588 support
  - drivers: can: Add TDC support
  - drivers: mchtmr: add counter init API.
  - drivers: dma: add dma_disable_channel() API
  - middleware: add wav decoder
  - samples: lwip: add ptp demos
  - samples: tinyusb: add msc, cdc demo
  - samples: audio_codec: add wav player demo
  - samples: add tinyuf2 initial support
  - samples: add initial uart dma rx idle demo

### Fixed:
  - soc: HPM6750: sysctl: fix cpu lp mode API.
  - drivers: uart: correct baudrate calculation.
  - drivers: usb: fix: no response in device mode when a USB cable is pluged out
  - boards: correct the pin setting related to USB

## [0.9.0] - 2022-04-01:
Main changes since 0.8.0

### Changed:
  - drivers: update WDG, UART, DMA, DMAMUX driver
  - drivers: enet remove enet_intf_selection
  - drivers: can: support configure bit timing via low-level bit timing parameters
  - drivers: optimize gpio driver
  - samples: exclude flash targets for mbx core1
  - samples: adjust SES project setting for coremark
  - samples: jpeg: update and integration jpeg decode samples
  - ses: use relpath for linker script in project file
  - ses: add HPM device name in generated project file
  - soc: HPM6750: add an interface selection api

### Added:
  - soc: HPM6750: add ram linker for core1
  - ses: support to use Andes toolchain
  - middleware: add hpm_math (replacing hpm_dsp)
  - samples: add lwip httpd
  - drivers: add section and alignment general instructions in hpm_common.h

### Fixed:
  - boards: hpm6750evk: fix bldc adc pinmux error
  - boards : hpm6750evk : pinmux : fix spi init pins error
  - samples: sdp: fix non-cacheable data initialization issue
  - samples: littlevgl: fix wrong picture patch in README

## [0.8.0] - 2022-03-04:
All changes since 0.7.3

### Changed:
  - rename middleware/sdmmc to middleware/hpm_sdmmc
  - place isr into .isr_vector section for irq non-vector mode
  - change csr functions to support llvm

### Fixed:
  - ses: fix issue in register xml
  - freertor: fix trap handler at non-vector mode
  - sdxc: fix software reset issue

### Added:
  - add sdk doc
  - add more sample doc
  - add multicore demo
  - i2c/uart: add dma support
  - add tensorflow lite for microcontroller

## [0.7.3] - 2022-02-23:
All changes since 0.7.2

### Changed:
  - freertos: change exception handling

## [0.7.2] - 2022-02-21:
All changes since 0.7.1

### Changed:
  - freertos: change freertos irq stack definition, passed in CMakeLists.txt, defined in linker
  - soc: hpm6750: add DISABLE_IRQ_PREEMPTIVE to check if it needs to enable irq preemption

### Fixed:
  - freertos: disable irq preemption

## [0.7.1] - 2022-02-13:

All changes since 0.7.0

### Changed:
  - drivers: adc12: update adc12_prd_config_t
  - samples: can: update case with interrupt and communication between two boards

### Fixed:
  - drivers: can: fix blocking transcation issue
  - samples: mbx: support run this example in SES
  - SES: startup: add fpu enable if abi is set to enable hw fp

### Added:
  - samples: multicore: add flash based multicore example
  - drivers: can: add apis to recvieve message for non-blocking use

## [0.7.0] - 2022-01-30:

All changes since 0.6.2

### Changed:
  - update default CPU frequency to 816MHz from 648MHz
  - update the ip headers
  - drivers: gpio: replace gpio_XXX_pins with gpio_XXX_port
  - drivers: gpio: remove pin level enum definition
  - drivers: i2s: driver update to remove mclk_div
  - drivers: ptpc: update driver
  - drivers: common: update get first set bit API
  - drivers: uart: split one enable with parameter into enable and disable interfaces
  - drivers: pwm: change name of output_channel config API
  - drivers: trgm: split separate API to enable/disable io output
  - soc: HPM6750: initialize noncacheable data in startup
  - soc: HPM6750: l1c: update fence.i call
  - samples: hello_world: add LED flashing
  - samples: littlevgl: remove lvgl example

### Fixed:
  - hpm6750evkmini: correct refresh cycle number of sdram
  - hpm6750evkmini: Fix the SDRAM memory range issue in flash_sdram_xip linker file
  - SES: remove no_relax option for linker, since segger has provide a patch to its ld for ses v6.10.
  - drivers: gpio: fix read pin issue:
  - drivers: usb: fix usb disconnection under linux environment
  - drivers: sdxc: Fixed the compatibility issue on different SD/eMMC cards in the sdcard_fatfs demo
  - drivers: gptmr: incorrect DMAEN configuration condition
  - drivers: gptmr: clear CNTRST bit after set.
  - SDK_DECLARE_EXT_ISR_M cannot work in the c++ file
  - FreeRTOS: fix ISR_STACK setting
  - components: touch: gt911: fix gpio write pin call

### Added:
  - SEG: add register definition file in generated embedded studio project
  - samples: drivers: gpiom: Add example to demonstrate gpiom's function
  - drivers: common: add macro to put data into noncacheable sections
  - middleware: integrate lwip
