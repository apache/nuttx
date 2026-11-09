NuttX-13.1.1
============

What's New In This Release
Major Changes to Core OS
Sched
* [#20596](https://github.com/apache/nuttx/pull/20596) sched: Add options for the argument limits of a new task
* [#20391](https://github.com/apache/nuttx/pull/20391) sched: Fix deadlock cycle collection.
* [#20420](https://github.com/apache/nuttx/pull/20420) sched/clock: keep the seqlock sequence in uint32_t
* [#20640](https://github.com/apache/nuttx/pull/20640) sched/signal: Run a synchronous signal handler out of the critical section
* [#20424](https://github.com/apache/nuttx/pull/20424) sched/signal: validate a sigevent's signal and target thread
* [#20543](https://github.com/apache/nuttx/pull/20543) sched/task: Do not log a missing or unloadable program as an exec error
* [#20651](https://github.com/apache/nuttx/pull/20651) sched/task: make task reparenting transactional
* [#20650](https://github.com/apache/nuttx/pull/20650) sched/task: record status for the terminated child
* [#20438](https://github.com/apache/nuttx/pull/20438) sched/timer: a timer handle works only in its own proces

MM
* [#20600](https://github.com/apache/nuttx/pull/20600) mm/mm_gran: Count granules in 32 bits.
* [#20629](https://github.com/apache/nuttx/pull/20629) mm/mm_gran: Do not skip free granules below a used bit outside the range
* [#20638](https://github.com/apache/nuttx/pull/20638) mm/umm_heap: Do not set errno in the kernel copy of the user heap

libc
* [#20489](https://github.com/apache/nuttx/pull/20489) libs/libc: Generate the system symbol tables in the CMake build.
* [#20550](https://github.com/apache/nuttx/pull/20550) libs/libc/arm64: Do not dereference a NULL symbol in up_reloateadd()
* [#20601](https://github.com/apache/nuttx/pull/20601) libs/libc/elf: Read the symbol and string tables at once to find a symbol
* [#20555](https://github.com/apache/nuttx/pull/20555) libs/libc/elf: Read the symbol table at once for RELA relocations.
* [#20597](https://github.com/apache/nuttx/pull/20597) libs/libc/elf: Return ENOEXEC for a file shorter than an ELF header
* [#20693](https://github.com/apache/nuttx/pull/20693) libs/libc/netdb: separate h_errno from EAI error codes
* [#20560](https://github.com/apache/nuttx/pull/20560) libs/libc/time: Set tm_zone to "UTC" in gmtime_r()
* [#20634](https://github.com/apache/nuttx/pull/20634) libs/libc/stdlib: Raise SIGABRT in abort()
* [#20633](https://github.com/apache/nuttx/pull/20633) libs/libc/stdio: Flush a line buffered stream at a newline in fwrite()
* [#20595](https://github.com/apache/nuttx/pull/20595) libs/libc/stdio: Close the descriptor in fclose() of a standard stream
* [#20440](https://github.com/apache/nuttx/pull/20440) libs/libc/termios: Fix the ttyname_r buffer overflow with long tty paths.
* [#20602](https://github.com/apache/nuttx/pull/20602) libs/libc/unistd: Return ARG_MAX from sysconf(_SC_ARG_MAX)
* [#20632](https://github.com/apache/nuttx/pull/20632) libs/libm: Compute frexp() and ldexp() exactly.
* [#20682](https://github.com/apache/nuttx/pull/20682) libs/libm: support 128-bit quad-precision and universal fallback in truncl

tools
* [#20663](https://github.com/apache/nuttx/pull/20663) tools/flash_writer.py: Replace removed telnetlib for Python3.13+

various
* [#20507](https://github.com/apache/nuttx/pull/20507) audio: Fix the build with CONFIG_AUDIO_MULTI_SESSION
* [#20409](https://github.com/apache/nuttx/pull/20409) binfmt: copy argv and file actions once, bounded
* [#20556](https://github.com/apache/nuttx/pull/20556) binfmt: Search PATH only for a file name without a slash.
* [#20468](https://github.com/apache/nuttx/pull/20468) binfmt: Update the TLS pid when exec() swaps the pids.
* [#20635](https://github.com/apache/nuttx/pull/20635) include: Fix sys/wait.h without stdbool.h, and add MAXPATHLEN and in6addr_loopback
* [#20594](https://github.com/apache/nuttx/pull/20594) include/elf.h: Define the ELF32 and ELF64 types on all targets
* [#20645](https://github.com/apache/nuttx/pull/20645) include/nuttx/sdio: guard callbackenable under HPWORK support
* [#20646](https://github.com/apache/nuttx/pull/20646) include/nuttx/vt100: fix macro syntax errors and add cursor movement
* [#20529](https://github.com/apache/nuttx/pull/20529) include/sys/types.h: Define wchar_t before including <stdint.h>
* [#20538](https://github.com/apache/nuttx/pull/20538) openamp: Do not trust buffer and header lengths written by the remote.
Major Changes to the Build System
Improvements
* [#20610](https://github.com/apache/nuttx/pull/20610) cmake: deduplicate NUTTX_EXTRA_FLAGS in linking stage

Architectural Support
Improvements
* [#20528](https://github.com/apache/nuttx/pull/20528) arch: Let pgalloc() extend the selected address environment
* [#20410](https://github.com/apache/nuttx/pull/20410) arch: refuse reserved syscalls from user space
* [#20056](https://github.com/apache/nuttx/pull/20056) arch/atomic: remove up_testset in spinloc

ARM
* [#20539](https://github.com/apache/nuttx/pull/20539) arch/arm: redirect an M-profile thread to its signal action once
* [#20457](https://github.com/apache/nuttx/pull/20457) arch/arm/imxrt: a fixed link when no PHY answers
* [#20477](https://github.com/apache/nuttx/pull/20477) arch/arm/imxrt: Fix nxstyle issues on imxrt_usdhc.c
* [#20562](https://github.com/apache/nuttx/pull/20562) arch/arm/imxrt: hold RX frames while the ENET TX ring is full
* [#20694](https://github.com/apache/nuttx/pull/20694) arch/arm/imxrt: imxrt_enet: fix YT8512 setup and add LAN8720 PHY notify
* [#20428](https://github.com/apache/nuttx/pull/20428) arch/arm/imxrt: lpspi fixes
* [#20587](https://github.com/apache/nuttx/pull/20587) arch/arm/rp23xx: add bounded timeout to SPI DMA completion wait
* [#20480](https://github.com/apache/nuttx/pull/20480) arch/arm/rp23xx: Align the flash MTD driver with the Pico SDK
* [#20503](https://github.com/apache/nuttx/pull/20503) arch/arm/rp23xx: Clear the DMA callback before invoking it
* [#20607](https://github.com/apache/nuttx/pull/20607) arch/arm/rp23xx: fix compilation error due to undeclared GPIO_FUNC_NULL
* [#20478](https://github.com/apache/nuttx/pull/20478) arch/arm/rp23xx: PSRAM QMI fixes from the Pico SDK
* [#20485](https://github.com/apache/nuttx/pull/20485) arch/arm/rp23xx: Use the right formats in the PWM period message
* [#20642](https://github.com/apache/nuttx/pull/20642) arch/arm/stm32: fix invalid device pointer in adc_sampletime_cfg
* [#20647](https://github.com/apache/nuttx/pull/20647) arch/arm/stm32h7: remove non-existent QSPI_CR_DMAEN references
ARM64
* [#20527](https://github.com/apache/nuttx/pull/20527) arch/arm64: Save d8-d15 in setjmp() and make jmp_buf hold them
* [#20402](https://github.com/apache/nuttx/pull/20402) arch/arm64: make per-core -mcpu flags reachable in both build systems
* [#20404](https://github.com/apache/nuttx/pull/20404) arch/arm64: bound pgalloc() to the user heap
* [#20406](https://github.com/apache/nuttx/pull/20406) arch/arm64/imx9: reject PWM channels outside the timer
* [#20510](https://github.com/apache/nuttx/pull/20510) arch/arm64: Report a recovered user fault as a segmentation fault
* [#20549](https://github.com/apache/nuttx/pull/20549) arch/arm64: Map a region that does not start at a page table boundary
* [#20606](https://github.com/apache/nuttx/pull/20606) arch/arm64: Do not list arm64_perf.o twice in the build rules
* [#20639](https://github.com/apache/nuttx/pull/20639) arch/arm64: Start a user task with a fixed SPSR
AVR
* [#20677](https://github.com/apache/nuttx/pull/20677) arch/AVR/DA/DB: fix for build failure and other smaller fixes
MIPS
* [#20450](https://github.com/apache/nuttx/pull/20450) arch/mips: record the running task on interrupt exit
* [#20449](https://github.com/apache/nuttx/pull/20449) arch/mips/pic32mz: do not exceed the requested SPI frequency
* [#20451](https://github.com/apache/nuttx/pull/20451) arch/mips/pic32mz: fix the Ethernet driver, add PHY ioctls and cache support
* [#20453](https://github.com/apache/nuttx/pull/20453) arch/mips/pic32mz: fix the GPIO_EDGE_RISING pin encoding
* [#20454](https://github.com/apache/nuttx/pull/20454) arch/mips/pic32mz: fix the L1 cache support
* [#20517](https://github.com/apache/nuttx/pull/20517) arch/mips/pic32mz: support promiscuous mode in the Ethernet driver
SIM
* [#20482](https://github.com/apache/nuttx/pull/20482) arch/sim: Drain all host CAN frames per RX work cycle.
* [#20563](https://github.com/apache/nuttx/pull/20563) arch/sim: drop the frame on a TAP write error
* [#20474](https://github.com/apache/nuttx/pull/20474) arch/sim: fix issues with CAN driver
RISC_V
* [#20509](https://github.com/apache/nuttx/pull/20509) arch/risc-v: Recover from a user fault without a kernel stack
* [#20626](https://github.com/apache/nuttx/pull/20626) arch/risc-v: Fix TLS size and initialization to use byte offsets>

X86_64
* [#20508](https://github.com/apache/nuttx/pull/20508) arch/x86_64: Kill only the offending task on a user-space fault

XTENSA
* [#20396](https://github.com/apache/nuttx/pull/20396) arch/espressif: stop leaking a Wi-Fi interrupt handle on every esp_wifi_st…
* [#20678](https://github.com/apache/nuttx/pull/20678) arch/xtensa/espressif: Include nuttx/kmalloc.h in esp_temperature_sensor

Driver Support
Improvements
* [#20505](https://github.com/apache/nuttx/pull/20505) drivers/audio: Make tone.h self-contained
* [#20657](https://github.com/apache/nuttx/pull/20657) drivers/can: repair CTUCANFD socket driver flag error
* [#20643](https://github.com/apache/nuttx/pull/20643) drivers/can/can.c: Fix FIONREAD result type and FIFO wrap
* [#20672](https://github.com/apache/nuttx/pull/20672) drivers/can/ctucanfd: Fix 29-bit extended CAN identifiers
* [#20673](https://github.com/apache/nuttx/pull/20673) drivers/can/ctucanfd_pci.c: fix interupt storm for SocketCAN
* [#20481](https://github.com/apache/nuttx/pull/20481) drivers/can/ctucanfd_pci.c: fix SocketCAN RX build and packet leak.
* [#20475](https://github.com/apache/nuttx/pull/20475) drivers/can/kvaser_pci.c: fix compilation error
* [#20572](https://github.com/apache/nuttx/pull/20572) drivers/i3c/master.c: fix dynamic address slot leak on reattach
* [#20614](https://github.com/apache/nuttx/pull/20614) drivers/net: e1000, igb, igc: drop RX frames when no packet is free
* [#20622](https://github.com/apache/nuttx/pull/20622) drivers/net: fix ABBA deadlock in netdev_upper_txavail under forwarding
* [#20636](https://github.com/apache/nuttx/pull/20636) drivers/net/{e1000|igb|igc}: various fixes found with automated tests
* [#20494](https://github.com/apache/nuttx/pull/20494) drivers/net/telnet: fix standard issues
* [#20506](https://github.com/apache/nuttx/pull/20506) drivers/power: Do not call gauge methods a driver did not implement
* [#20670](https://github.com/apache/nuttx/pull/20670) drivers/serial/uart: Fix missing semicolon in u16550_dmarxfree
* [#20591](https://github.com/apache/nuttx/pull/20591) drivers/serial/uart_pl011.c: Send the TX buffer with interrupts disabled
* [#20397](https://github.com/apache/nuttx/pull/20397) drivers/timers/pcf85263: report a lost time as lost.
* [#20884](https://github.com/apache/nuttx/pull/20884) drivers/usbhost/cdcmbim: validate device-provided NTB bounds - #20884
* [#20675](https://github.com/apache/nuttx/pull/20675) drivers/virtio: Take RX buffers used while re-enabling the callback

Board Support
Improvements
* [#20661](https://github.com/apache/nuttx/pull/20661) boards/arm/linum-stm32h753bi: Include unistd.h for usleep()
* [#20486](https://github.com/apache/nuttx/pull/20486) boards/arm/rp23xx: Do not log an absent SD card as an error.
* [#20577](https://github.com/apache/nuttx/pull/20577) boards/arm/stm32g4: enable LTO on cansock configurations
* [#20604](https://github.com/apache/nuttx/pull/20604) boards/arm/stm32h7: nucleo-h743zi: fix implicit declaration of cdcacm_init
* [#20554](https://github.com/apache/nuttx/pull/20554) boards/arm/tiva/lm3s6965-ek: Drop wget from qemu-flat to free flash

File System
Improvements
* [#20631](https://github.com/apache/nuttx/pull/20631) fs: Set the mode that chmod() gives, on v9fs and in the VFS
* [#20416](https://github.com/apache/nuttx/pull/20416) fs/mmap: back user anonymous mappings with pages in a kernel build
* [#20446](https://github.com/apache/nuttx/pull/20446) fs/smartfs: mask reserved flags from stat mode.
* [#20499](https://github.com/apache/nuttx/pull/20499) fs/v9fs: Fix the parsing of the mount options
* [#20525](https://github.com/apache/nuttx/pull/20525) fs/v9fs: Report the end of a directory and set st_ino and d_ino
* [#20548](https://github.com/apache/nuttx/pull/20548) fs/v9fs: Split virtio buffers where their physical pages are not adjcent
* [#20630](https://github.com/apache/nuttx/pull/20630) fs/v9fs: Wait for the reply even if a signal wakes the caller
* [#20559](https://github.com/apache/nuttx/pull/20559) fs/vfs: Make fstat() of an open directory describe the directory.
* [#20648](https://github.com/apache/nuttx/pull/20648) fs/vfs: Report POLLERR in the read and write sets of select()

Networking
Improvements
* [#20617](https://github.com/apache/nuttx/pull/20617) net/can: Honor SO_RCVTIMEO in can_recvmsg()
* [#20615](https://github.com/apache/nuttx/pull/20615) net/can/can_callback.c: Fix the readahead warning format
* [#20888](https://github.com/apache/nuttx/pull/20888) net/icmpv6: validate router advertisement option bounds
* [#20885](https://github.com/apache/nuttx/pull/20885) net/ipv6: validate extension header bounds
* [#20886](https://github.com/apache/nuttx/pull/20886) net/mld: validate MLDv2 report record bounds
* [#20674](https://github.com/apache/nuttx/pull/20674) net/tcp: Do not send more than the MSS when IOBs run low
* [#20641](https://github.com/apache/nuttx/pull/20641) net/tcp: fix use-after-free race and nullptr dereference in tcp_timer
* [#20552](https://github.com/apache/nuttx/pull/20552) net/tcp: reuse closing connections also with CONFIG_NET_SOLINGER
* [#20613](https://github.com/apache/nuttx/pull/20613) net/tcp: skip random port init for fixed-port bind.
* [#21101](https://github.com/apache/nuttx/pull/21101) net/usbhost/fs: Harden malformed input handling
* [#20588](https://github.com/apache/nuttx/pull/20588) net/w5500: tolerate spurious events and avoid permanent fencing
* [#20688](https://github.com/apache/nuttx/pull/20688) wireless/bluetooth: Clear the sync reference when a command times out
