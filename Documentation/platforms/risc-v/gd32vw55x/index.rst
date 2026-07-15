====================
GigaDevice GD32VW55x
====================

The GD32VW553 is a Wi-Fi 6 (802.11 b/g/n/ax, 1x1) and Bluetooth LE 5.3 combo
SoC built around a Nuclei N307 RISC-V core.  The radio MAC/PHY, RF and BLE
controller are provided by the vendor as prebuilt BSD-3-licensed libraries;
the open layers of the vendor SDK (Wi-Fi manager, platform BSP, standard
peripheral library) are rebuilt from source.

- Nuclei N307 RISC-V core (RV32IMAFC), 160 MHz, single-precision FPU
- ECLIC interrupt controller, 64-bit machine timer
- 320 KB SRAM, up to 4096 KB internal flash
- Wi-Fi 6 (802.11 b/g/n/ax) 1x1, 20 MHz
- Bluetooth LE 5.3 (RivieraWaves controller)
- Wi-Fi and BLE coexistence arbitration in hardware

Supported chips
===============

The family differs in flash size and package only; all variants share the
same peripheral set and are selected under "GD32VW55x Chip Selection":

============== ========= ===== ========= ========
Chip           Package   GPIO  Flash     SRAM
============== ========= ===== ========= ========
GD32VW553KIQ   QFN32     22    2048 KB   320 KB
GD32VW553KMQ   QFN32     22    4096 KB   320 KB
GD32VW553HIQ   QFN40     29    2048 KB   320 KB
GD32VW553HMQ   QFN40     29    4096 KB   320 KB
============== ========= ===== ========= ========

Toolchain
=========

Any RISC-V GCC with an ``rv32imafc`` / ``ilp32f`` multilib works.  The
prebuilt vendor libraries are built for the **hard-float single-precision
ABI** (``-mabi=ilp32f``); a soft-float toolchain cannot link them.

The xPack ``riscv-none-elf-gcc`` used by the NuttX CI (14.2.0) provides that
multilib and is the recommended toolchain.  NuttX selects it automatically
when it is on ``PATH``; no ``CROSSDEV`` setting is needed.

Vendor SDK
==========

The Wi-Fi and BLE support needs the GigaDevice GD32VW55x Wi-Fi/BLE SDK.  The
build clones it automatically during ``make context``, pinned to a validated
commit, and applies the port patches::

  https://github.com/GigaDeviceSemiconductor/GD32VW55x_WiFi_BLE_SDK

The pinned revision is kept in ``arch/risc-v/src/gd32vw55x/gdwifi/Wireless.mk``
(SDK V1.0.3g).  The port depends on the ABI of the prebuilt libraries -- do
not move the pin without revalidating.  ``CONFIG_GD32VW55X_WIFI_SDK_PATH`` may
point at a local checkout instead, for development.

Floating Point
==============

The N307 core has a **single-precision** hardware FPU (F extension, FLEN=32).
There is no hardware double precision: ``double`` is emulated in software, so
prefer ``float`` in hot paths.  ``CONFIG_ARCH_FPU`` is enabled;
``CONFIG_ARCH_DPFPU`` does not apply to this core.

The silicon also implements the B (bit manipulation) and P (packed SIMD, draft
0.94) extensions.  The prebuilt vendor libraries are compiled with them
(``rv32imafcbp``), and the SDK ships an NMSIS DSP library built for the same
ISA.  NuttX itself is compiled as ``rv32imafc`` -- the kernel does not use B/P
-- and links cleanly against those libraries.

DMA
===

The SoC has a single DMA controller with **8 channels**; each channel selects
one of 8 peripheral request lines (sub-peripherals), and supports circular
mode, burst transfers, a FIFO and memory-to-memory transfers.

The Wi-Fi and BLE data paths use DMA internally, inside the prebuilt
libraries, and own the channels they need.  The peripheral drivers claim
their channels through ``gd32_dma_channel_alloc()``; the SPI driver uses it
under ``CONFIG_GD32VW55X_SPI_DMA``.

Memory map
==========

============ =========================== ========= =============================
Region       Range                       Size      Purpose
============ =========================== ========= =============================
Flash        0x08000000 -- 0x08400000    4 MiB     In-package NOR, XIP.  Holds
                                                   the firmware and, at the very
                                                   top, the Wi-Fi NVDS
Mask ROM     0x0bf40000 -- 0x0bf80000    256 KiB   Boot ROM (IBL) and the ROM
                                                   API used to program the flash
EFUSE        0x0ffc0000 -- 0x0ffc0100    256 B     Chip data; the radio reads
                                                   the MAC address from here
SRAM         0x20000000 -- 0x20050000    320 KiB   See the split below
BLE RAM      0x21000000 -- 0x21010000    64 KiB    Exchange memory of the BLE
                                                   controller
QSPI window  0x90000000 -- ...           --        XIP window of an optional
                                                   external flash
Peripherals  0x40000000 -- 0x4c0fffff    --        APB1/APB2/AHB1, the Wi-Fi MAC
                                                   at 0x40030000, and the crypto
                                                   accelerators on AHB2
Core-local   0xd1000000 / 0xd2000000     --        Nuclei SysTimer and ECLIC
============ =========================== ========= =============================

Flash
-----

The NuttX image is linked at the start of the flash: the vendor MBL bootloader
is bypassed, so none of its partitions (system status, image slots) apply.

Only one region of the flash is fixed by the silicon and its radio firmware:

======================== ========= ==========================================
Range                    Size      Purpose
======================== ========= ==========================================
0x08000000 -- 0x083fb000 4076 KiB  Usable by the board: the firmware, and
                                   whatever else the board decides to carve
                                   out of it (progmem, a filesystem, an OTA
                                   slot...)
0x083fb000 -- 0x08400000 20 KiB    **Wi-Fi NVDS**: RF calibration data and the
                                   MAC address.  Erasing it breaks the radio
======================== ========= ==========================================

.. warning::
   Never erase past 0x083fb000.  ``wapi scan wlan0`` is the quickest way to
   confirm the radio is still healthy after touching the flash.

How the 4076 KiB below the NVDS are split is a **board decision**, made in the
board's linker script and in ``CONFIG_GD32VW55X_PROGMEM_START_ADDR`` /
``CONFIG_GD32VW55X_PROGMEM_SIZE``.  See the board documentation for the layout
that board uses.

SRAM
----

======================== ========= ==========================================
Range                    Size      Purpose
======================== ========= ==========================================
0x20000000 -- 0x20000200 512 B     Runtime data of the mask ROM (IBL state,
                                   ROM mbedTLS).  Must stay untouched
0x20000200 -- 0x20048000 287.5 KiB The application: NuttX links here
0x20048000 -- 0x20050000 32 KiB    Shared SRAM: RX buffers of the Wi-Fi
                                   firmware
======================== ========= ==========================================

This is why the linker script both raises the origin and shrinks the length by
0x200 (``ORIGIN = 0x20000200, LENGTH = 288K - 0x200``): the region has to start
above the mask ROM data *and* still end at 0x20048000, where the Wi-Fi shared
SRAM begins.  Growing it by those 512 bytes would put the heap on top of the
MAC's RX buffers.

Internal flash
==============

The internal flash is a **system-in-package NOR die**, and it is *not*
programmed through the FMC registers.  The vendor code always goes through
the mask ROM (``rom_flash_read``/``write``/``erase``), and so does this port.

Poking ``FMC_CTL`` directly does erase the array, but the programmed data
never reads back: the flash sits behind the real-time decryption block
(RTDEC), which the ROM routines handle and a register-level driver does not.
The ROM API table is at a fixed address, so ``rom_init()`` is not needed just
to use it.

The ROM accepts any length, so writes have no 32-bit alignment constraint.

Peripheral Support
==================

The table lists every peripheral that exists on the GD32VW55x family.  Note
that this family has **no I2S, USB, CAN, DAC, SDIO or Ethernet** controller.

============== ======= ==================================================
Peripheral     Support Notes
============== ======= ==================================================
ECLIC          Yes     Nuclei interrupt controller, 116 sources
RCU/Clock tree Yes     160 MHz core / 40 MHz HXTAL
SysTimer       Yes     64-bit machine timer
USART/UART     Yes     USART0 (flow control), UART1, UART2
GPIO           Yes     Ports A, B and C
EXTI           Yes     Pin interrupts, 16 lines (5..9 and 10..15 shared)
DMA            Yes     8 channels, 8 request lines each, FIFO and burst
SPI            Yes     One instance, optional DMA
I2C            Yes     I2C0 and I2C1, up to 1 MHz
ADC            Yes     One instance, 11 channels, 6/8/10/12-bit.  A
                       routine sequence takes at most 9 channels
PWM            Yes     TIMER0/1/2/15/16.  TIMER0 has complementary
                       outputs with dead-time insertion
Input capture  Yes     TIMER0/1/2 (needs two channels and the slave-mode
                       controller)
FWDGT          Yes     Independent watchdog, IRC32K
WWDGT          Yes     Window watchdog, APB1
PROGMEM        Yes     Internal flash, 4 KiB page erase, through the
                       mask ROM (see below)
TRNG           Yes     /dev/random, /dev/urandom
CRC            Yes     Hardware CRC-32 unit
RTC            Partial Calendar over IRC32K.  Alarm, tamper, timestamp
                       and wakeup are not implemented yet
EFUSE          Partial Read-only, used by the radio for the MAC address
Wi-Fi          Yes     Station and softAP (one at a time), WPA2, ``wlan0``
BLE            Exp.    RivieraWaves controller + vendor host: advertising,
                       connection and a demo GATT service (see the board doc)
TIMER5         No      Basic timer (no channels): no time-base driver yet
QSPI           No      Quad-SPI, for external flash
CAU/HAU/PKCAU  No      Crypto accelerators
PMU            No      Low-power modes and LVD.  Wi-Fi power save is
                       disabled at bring-up
============== ======= ==================================================

Timers
------

============ ======= ========== ============== =========== ========
Timer        Counter Channels   Complementary  Slave mode  Capture
============ ======= ========== ============== =========== ========
TIMER0       16-bit  CH0..CH3   CH0N..CH2N     Yes         Yes
TIMER1       32-bit  CH0..CH3   --             Yes         Yes
TIMER2       32-bit  CH0..CH3   --             Yes         Yes
TIMER5       16-bit  --         --             No          No
TIMER15      16-bit  CH0        CH0N           No          No
TIMER16      16-bit  CH0        CH0N           No          No
============ ======= ========== ============== =========== ========

All timers are clocked at 160 MHz (``RCU_CFG1.TIMERSEL`` at its reset value,
with the prescalers this port uses).

Unlike the ST parts this IP derives from, the **slave mode and the trigger
source are not in ``TIMER_SMCFG``**: they live in ``SYSCFG_TIMERxCFG``, as
eight 4-bit fields indexed by slave mode.

The crypto accelerators (CAU, HAU, PKCAU) are reachable by the radio through
the vendor SDK, but are not exposed to NuttX (``/dev/crypto``) yet.

Wi-Fi
=====

The MAC/PHY firmware (``libwifi.a``), the RF library (``librf.a``) and the WPA
supplicant (``libwpas.a``) are linked as prebuilt libraries.  The OS binding is
``gdwifi/wrapper_nuttx.c``, which implements the SDK's ``sys_*`` OS facade on
top of NuttX primitives -- the prebuilt libraries are RTOS-agnostic and
reference no FreeRTOS symbol.

The lwIP stack shipped with the SDK is **not** used: the interface is a
``netdev_lowerhalf`` driver registered as ``wlan0``, driven by the standard
NuttX tools (``ifup``, ``wapi``, ``renew``, ``ping``).

BLE
===

The prebuilt ``libble_max.a`` is an all-in-one controller plus RivieraWaves
host: GAP, GATT and SMP live inside the blob and the controller talks to them
internally, not over HCI (the library carries no HCI transport -- its ``h4tl``
object is empty).  A NuttX ``bt_driver_s`` over HCI therefore cannot drive it,
so the port calls the vendor host directly (``ble_power_on`` -> ``ble_sw_init``
-> ``ble_adp_*`` -> ``ble_adv_*``), the way the GigaDevice BLE examples do.

The SDK's radio interrupt handlers must be attached to the NuttX IRQ table:
the vendor firmware reaches them through the ECLIC hardware vector table, while
NuttX dispatches through the common trap handler, so an enabled-but-unattached
source lands in ``irq_unexpected_isr``.  The ``irq_attach()`` call is also the
only reference that keeps ``--gc-sections`` from dropping those handlers.

It is EXPERIMENTAL and off by default.  When enabled it advertises a
connectable set named ``NuttX``.

Coexistence between Wi-Fi and BLE is arbitrated inside the prebuilt
libraries; the port only registers the BLE scheduler callback with the Wi-Fi
MAC.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
