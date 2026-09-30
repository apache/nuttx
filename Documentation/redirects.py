# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.

"""Redirect map for pages that were moved or merged.

Every documentation page that is renamed, moved or merged into another one
MUST get an entry here.  The NuttX documentation is linked to from issues,
mailing list archives, blog posts and slide decks going back many years;
without these redirects a reorganisation silently breaks all of them.

The mapping is consumed by the ``sphinx_reredirects`` extension and is
keyed by the *old* document name (no extension, path relative to the
documentation root).  The value is the *new* location, relative to the old
page's directory -- see the sphinx-reredirects documentation.

Keep the entries grouped by the change that introduced them and keep each
group sorted, so that the file stays reviewable as it grows.
"""

import pathlib
import posixpath


def _moved(pairs):
    """Turn (old docname, new docname) pairs into a sphinx-reredirects map.

    sphinx-reredirects expects the target to be written relative to the
    directory the *old* page used to live in, which is easy to get wrong by
    hand.  Spell the moves out as document names and let posixpath work the
    relative link out.
    """
    out = {}
    for old, new in pairs:
        assert old not in out, f"duplicate redirect for {old}"
        out[old] = posixpath.relpath(new, posixpath.dirname(old)) + ".html"
    return out


# Aligning Documentation/platforms/ with the source tree.
#
#   * the architecture directory was spelled "misco" instead of "misoc";
#   * the chip directory "ra4m1" is "ra4" under arch/ and boards/;
#   * LPC17xx and LPC40xx are one directory in the source tree
#     (arch/arm/src/lpc17xx_40xx, boards/arm/lpc17xx_40xx) but were two here;
#   * four boards were filed under a name the source tree does not use.
_PLATFORM_ALIGNMENT = [
    # misco -> misoc
    ("platforms/misco/index", "platforms/misoc/index"),
    ("platforms/misco/lm32/index", "platforms/misoc/lm32/index"),
    (
        "platforms/misco/lm32/boards/misoc/index",
        "platforms/misoc/lm32/boards/misoc/index",
    ),
    # ra4m1 -> ra4
    ("platforms/arm/ra4m1/index", "platforms/arm/ra4/index"),
    (
        "platforms/arm/ra4m1/boards/arduino-r4-minima/index",
        "platforms/arm/ra4/boards/arduino-r4-minima/index",
    ),
    (
        "platforms/arm/ra4m1/boards/xiao-ra4m1/index",
        "platforms/arm/ra4/boards/xiao-ra4m1/index",
    ),
    # lpc17xx -> lpc17xx_40xx
    ("platforms/arm/lpc17xx/index", "platforms/arm/lpc17xx_40xx/index"),
    (
        "platforms/arm/lpc17xx/boards/lincoln60/index",
        "platforms/arm/lpc17xx_40xx/boards/lincoln60/index",
    ),
    (
        "platforms/arm/lpc17xx/boards/lpcxpresso-lpc1768/index",
        "platforms/arm/lpc17xx_40xx/boards/lpcxpresso-lpc1768/index",
    ),
    (
        "platforms/arm/lpc17xx/boards/mbed/index",
        "platforms/arm/lpc17xx_40xx/boards/mbed/index",
    ),
    (
        "platforms/arm/lpc17xx/boards/mcb1700/index",
        "platforms/arm/lpc17xx_40xx/boards/mcb1700/index",
    ),
    (
        "platforms/arm/lpc17xx/boards/olimex-lpc1766stk/index",
        "platforms/arm/lpc17xx_40xx/boards/olimex-lpc1766stk/index",
    ),
    (
        "platforms/arm/lpc17xx/boards/open1788/index",
        "platforms/arm/lpc17xx_40xx/boards/open1788/index",
    ),
    (
        "platforms/arm/lpc17xx/boards/pnev5180b/index",
        "platforms/arm/lpc17xx_40xx/boards/pnev5180b/index",
    ),
    (
        "platforms/arm/lpc17xx/boards/u-blox-c027/index",
        "platforms/arm/lpc17xx_40xx/boards/u-blox-c027/index",
    ),
    (
        "platforms/arm/lpc17xx/boards/zkit-arm-1769/index",
        "platforms/arm/lpc17xx_40xx/boards/zkit-arm-1769/index",
    ),
    # lpc40xx -> lpc17xx_40xx
    ("platforms/arm/lpc40xx/index", "platforms/arm/lpc17xx_40xx/lpc40xx"),
    (
        "platforms/arm/lpc40xx/boards/lx_cpu/index",
        "platforms/arm/lpc17xx_40xx/boards/lx_cpu/index",
    ),
    # added upstream after the fork, filed here by where its code lives
    ("implementation/chroot", "os/filesystem/chroot"),
    # upstream typo: the page was named intex.rst
    (
        "platforms/arm/nrf52/boards/nrf52-feather/intex",
        "platforms/arm/nrf52/boards/nrf52-feather/index",
    ),
    # boards filed under a name the source tree does not use
    (
        "platforms/arm/stm32f1/boards/hymini-stm32/index",
        "platforms/arm/stm32f1/boards/hymini-stm32v/index",
    ),
    (
        "platforms/arm/mps/boards/mps2-an512/index",
        "platforms/arm/mps/boards/mps2-an521/index",
    ),
    (
        "platforms/mips/jz4780/boards/mips-creator-ci20/index",
        "platforms/mips/jz4780/boards/ci20/index",
    ),
    (
        "platforms/tricore/tc397/boards/kit_a2g_tc397_tft/index",
        "platforms/tricore/tc397/boards/a2g-tc397-5v-tft/index",
    ),
]

# Normalising the tag vocabulary.  Tag pages have stable URLs under _tags/,
# so a renamed tag needs a redirect like any other page.  Tags that were
# dropped rather than renamed point at the overview.
_TAG_VOCABULARY = [
    # The architecture is the directory: arch/risc-v, boards/risc-v.
    ("_tags/arch-riscv", "_tags/arch-risc-v"),
    # Lower case, like arch/risc-v/src/esp32c3.
    ("_tags/chip-esp32-c3", "_tags/chip-esp32c3"),
    # Tiva is a Texas Instruments product line, not a vendor.
    ("_tags/vendor-tiva", "_tags/vendor-ti"),
    # Peripherals and port maturity stopped being tags altogether; see
    # _DROPPED_FACTS below.
    ("_tags/ethernet", "_tags/tagsindex"),
    ("_tags/wifi", "_tags/tagsindex"),
    ("_tags/experimental", "_tags/tagsindex"),
    # Dropped: armv8-m and cortex-m33 are cores, not architectures, and the
    # pages that carried them already carry arch:arm.
    ("_tags/arch-armv8m", "_tags/tagsindex"),
    ("_tags/arch-cm33", "_tags/tagsindex"),
    # Dropped: these came from the board template, which no longer emits
    # live tags of its own.
    ("_tags/arch-example", "_tags/tagsindex"),
    ("_tags/chip-example", "_tags/tagsindex"),
    ("_tags/vendor-example", "_tags/tagsindex"),
]

# Splitting part numbers out of chip:.  chip: is now exactly the set of chip
# family directories; the individual part on a board moved to part:.
_CHIP_TO_PART = [
    ("_tags/chip-cxd5602", "_tags/part-cxd5602"),
    ("_tags/chip-esp32a1s", "_tags/part-esp32a1s"),
    ("_tags/chip-esp32picod4", "_tags/part-esp32picod4"),
    ("_tags/chip-esp32wroom32", "_tags/part-esp32wroom32"),
    ("_tags/chip-esp32wrover32", "_tags/part-esp32wrover32"),
    ("_tags/chip-fpga", "_tags/part-fpga"),
    ("_tags/chip-ht32f49163", "_tags/part-ht32f49163"),
    ("_tags/chip-imx93", "_tags/part-imx93"),
    ("_tags/chip-imx95", "_tags/part-imx95"),
    ("_tags/chip-nrf52832", "_tags/part-nrf52832"),
    ("_tags/chip-nrf52840", "_tags/part-nrf52840"),
    ("_tags/chip-nrf5340", "_tags/part-nrf5340"),
    ("_tags/chip-nrf9160", "_tags/part-nrf9160"),
    ("_tags/chip-rp2350", "_tags/part-rp2350"),
    ("_tags/chip-rp2350b", "_tags/part-rp2350b"),
    ("_tags/chip-stm32c071", "_tags/part-stm32c071"),
    ("_tags/chip-stm32c092", "_tags/part-stm32c092"),
    ("_tags/chip-stm32c562", "_tags/part-stm32c562"),
    ("_tags/chip-stm32f051", "_tags/part-stm32f051"),
    ("_tags/chip-stm32f072", "_tags/part-stm32f072"),
    ("_tags/chip-stm32f091", "_tags/part-stm32f091"),
    ("_tags/chip-stm32f100", "_tags/part-stm32f100"),
    ("_tags/chip-stm32f103", "_tags/part-stm32f103"),
    ("_tags/chip-stm32f107", "_tags/part-stm32f107"),
    ("_tags/chip-stm32f205", "_tags/part-stm32f205"),
    ("_tags/chip-stm32f207", "_tags/part-stm32f207"),
    ("_tags/chip-stm32f302", "_tags/part-stm32f302"),
    ("_tags/chip-stm32f303", "_tags/part-stm32f303"),
    ("_tags/chip-stm32f334", "_tags/part-stm32f334"),
    ("_tags/chip-stm32f401", "_tags/part-stm32f401"),
    ("_tags/chip-stm32f405", "_tags/part-stm32f405"),
    ("_tags/chip-stm32f407", "_tags/part-stm32f407"),
    ("_tags/chip-stm32f411", "_tags/part-stm32f411"),
    ("_tags/chip-stm32f412", "_tags/part-stm32f412"),
    ("_tags/chip-stm32f427", "_tags/part-stm32f427"),
    ("_tags/chip-stm32f429", "_tags/part-stm32f429"),
    ("_tags/chip-stm32f446", "_tags/part-stm32f446"),
    ("_tags/chip-stm32f722", "_tags/part-stm32f722"),
    ("_tags/chip-stm32f746", "_tags/part-stm32f746"),
    ("_tags/chip-stm32f767", "_tags/part-stm32f767"),
    ("_tags/chip-stm32f769", "_tags/part-stm32f769"),
    ("_tags/chip-stm32f777", "_tags/part-stm32f777"),
    ("_tags/chip-stm32g070", "_tags/part-stm32g070"),
    ("_tags/chip-stm32g071", "_tags/part-stm32g071"),
    ("_tags/chip-stm32g431", "_tags/part-stm32g431"),
    ("_tags/chip-stm32g474", "_tags/part-stm32g474"),
    ("_tags/chip-stm32h503", "_tags/part-stm32h503"),
    ("_tags/chip-stm32h533", "_tags/part-stm32h533"),
    ("_tags/chip-stm32h563", "_tags/part-stm32h563"),
    ("_tags/chip-stm32h723", "_tags/part-stm32h723"),
    ("_tags/chip-stm32h743", "_tags/part-stm32h743"),
    ("_tags/chip-stm32h745", "_tags/part-stm32h745"),
    ("_tags/chip-stm32h747", "_tags/part-stm32h747"),
    ("_tags/chip-stm32h750", "_tags/part-stm32h750"),
    ("_tags/chip-stm32h753", "_tags/part-stm32h753"),
    ("_tags/chip-stm32h7s3", "_tags/part-stm32h7s3"),
    ("_tags/chip-stm32l053", "_tags/part-stm32l053"),
    ("_tags/chip-stm32l072", "_tags/part-stm32l072"),
    ("_tags/chip-stm32l073", "_tags/part-stm32l073"),
    ("_tags/chip-stm32l152", "_tags/part-stm32l152"),
    ("_tags/chip-stm32l432", "_tags/part-stm32l432"),
    ("_tags/chip-stm32l452", "_tags/part-stm32l452"),
    ("_tags/chip-stm32l475", "_tags/part-stm32l475"),
    ("_tags/chip-stm32l476", "_tags/part-stm32l476"),
    ("_tags/chip-stm32l496", "_tags/part-stm32l496"),
    ("_tags/chip-stm32l4r9", "_tags/part-stm32l4r9"),
    ("_tags/chip-stm32l552", "_tags/part-stm32l552"),
    ("_tags/chip-stm32l562", "_tags/part-stm32l562"),
    ("_tags/chip-stm32n657", "_tags/part-stm32n657"),
    ("_tags/chip-stm32u083", "_tags/part-stm32u083"),
    ("_tags/chip-stm32u3c5", "_tags/part-stm32u3c5"),
    ("_tags/chip-stm32u585", "_tags/part-stm32u585"),
    ("_tags/chip-stm32u5a5", "_tags/part-stm32u5a5"),
    ("_tags/chip-stm32wb55", "_tags/part-stm32wb55"),
    ("_tags/chip-stm32wl55", "_tags/part-stm32wl55"),
    ("_tags/chip-tm4c123", "_tags/part-tm4c123"),
    ("_tags/chip-ultrascale", "_tags/part-ultrascale"),
    ("_tags/chip-virt", "_tags/part-virt"),
    ("_tags/chip-xczu28dr", "_tags/part-xczu28dr"),
]

# Dropped when the vocabulary was tightened: chip:stm32, chip:stm32wl and
# chip:zynq were product lines rather than families, and every page carrying
# them already carried its family.  "Every STM32 board" is vendor:st now.
_DROPPED_ROLLUPS = [
    ("_tags/chip-stm32", "_tags/vendor-st"),
    ("_tags/chip-stm32wl", "_tags/chip-stm32wl5"),
    ("_tags/chip-zynq", "_tags/chip-zynq-mpsoc"),
]

# vendor: now names who makes the chip and is derived from the chip family.
# The board makers it used to hold are already in the board names.
_DROPPED_BOARD_VENDORS = [
    ("_tags/vendor-arduino", "_tags/tagsindex"),
    ("_tags/vendor-beagleboard", "_tags/tagsindex"),
    ("_tags/vendor-elegoo", "_tags/tagsindex"),
    ("_tags/vendor-mikroelektronika", "_tags/tagsindex"),
    ("_tags/vendor-pine64", "_tags/tagsindex"),
    ("_tags/vendor-raspberry-pi", "_tags/tagsindex"),
    ("_tags/vendor-sipeed", "_tags/tagsindex"),
    ("_tags/vendor-xunlong", "_tags/tagsindex"),
    ("_tags/vendor-vega", "_tags/vendor-nxp"),
    # Freescale became part of NXP in 2015.
    ("_tags/vendor-freescale", "_tags/vendor-nxp"),
]

# What a board offers, and how far its port has been taken, are facts about
# the hardware.  As tags they covered a twentieth of the boards that actually
# have the peripheral, which reads as a complete answer and is not one; and
# every page tagged status:experimental already said so in its own text.  Both
# now live in the Support Status and Peripheral Support sections of the board
# page, where there is room to be exact.
_DROPPED_FACTS = [
    ("_tags/peripheral-dac", "_tags/tagsindex"),
    ("_tags/peripheral-ethernet", "_tags/tagsindex"),
    ("_tags/peripheral-wifi", "_tags/tagsindex"),
    ("_tags/status-experimental", "_tags/tagsindex"),
]

# Two directories that only the source tree could settle.  tricore/tc4d9 was a
# misspelling of arch/tricore/src/tc4da (the page itself is titled TC4DA), and
# avr/atmega128, atmega1284p and atmega2560 are parts of the atmega family
# rather than families of their own -- arch/avr/src/ has one atmega directory.
# The pages stay where they are, because they carry real per part content;
# only the tags were wrong.
_SOURCE_TREE_TRUTH = [
    ("platforms/tricore/tc4d9/index", "platforms/tricore/tc4da/index"),
    (
        "platforms/tricore/tc4d9/boards/triboard_tc4x9_com/index",
        "platforms/tricore/tc4da/boards/triboard_tc4x9_com/index",
    ),
    ("_tags/chip-tc4d9", "_tags/chip-tc4da"),
    ("_tags/chip-atmega128", "_tags/part-atmega128"),
    ("_tags/chip-atmega1284p", "_tags/part-atmega1284p"),
    ("_tags/chip-atmega2560", "_tags/part-atmega2560"),
]

# boards/x86_64/qemu/qemu-intel64/ is where the source tree files this board,
# so that is where its page belongs.  platforms/x86_64/intel64/ stays: it is a
# real family under arch/x86_64/src/, it just has no boards of its own.
_X86_64_BOARD = [
    (
        "platforms/x86_64/intel64/boards/qemu-intel64/index",
        "platforms/x86_64/qemu/boards/qemu-intel64/index",
    ),
]

# Grouping the OS documentation by subsystem instead of by how deep it goes.
# The scheduler pages were spread over implementation/ and reference/os/;
# they live under os/scheduling/ now.  reference/os/smp.rst is merged into
# os/scheduling/smp.rst, which is where its technical description already
# was.
_OS_SCHEDULING = [
    ("implementation/nuttx_tasking", "os/scheduling/nuttx_tasking"),
    ("implementation/tasks_vs_threads", "os/scheduling/tasks_vs_threads"),
    ("implementation/processes_vs_tasks", "os/scheduling/processes_vs_tasks"),
    (
        "implementation/kernel_threads_vs_pthreads",
        "os/scheduling/kernel_threads_vs_pthreads",
    ),
    ("implementation/context_switches", "os/scheduling/context_switches"),
    ("implementation/preemption_latency", "os/scheduling/preemption_latency"),
    ("implementation/cancellation_points", "os/scheduling/cancellation_points"),
    ("implementation/smp", "os/scheduling/smp"),
]

# Grouping the rest of the OS documentation by subsystem.  components/,
# implementation/ and reference/os/ described the same subsystems at three
# different depths; they are one tree now, following the source tree.
_OS_TREES = [
    ("components/drivers", "os/drivers"),
    ("components/filesystem", "os/filesystem"),
    ("components/net", "os/networking"),
    ("components/mm", "os/memory"),
    ("components/libs", "os/libs"),
    ("components/nxgraphics", "os/graphics"),
    ("components/audio", "os/audio"),
    ("components/arch", "os/arch"),
    ("components/concurrency", "os/concurrency"),
]

_OS_PAGES = [
    ("components/binfmt", "os/binfmt/index"),
    ("components/nxflat", "os/binfmt/nxflat"),
    ("components/crypto", "os/crypto"),
    ("components/video", "os/video"),
    ("components/wireless", "os/wireless"),
    ("components/syscall", "os/syscall"),
    ("components/paging", "os/memory/paging"),
    ("components/openamp", "os/openamp"),
    ("implementation/device_drivers", "os/drivers/device_drivers"),
    ("implementation/device_nodes", "os/drivers/device_nodes"),
    ("implementation/drivers_design", "os/drivers/drivers_design"),
    ("implementation/ioctl", "os/drivers/ioctl"),
    ("implementation/usb", "os/drivers/usb"),
    ("implementation/power_management", "os/drivers/special/power/power_management"),
    ("implementation/syslog", "os/drivers/special/syslog_design"),
    ("implementation/file_descriptors", "os/filesystem/file_descriptors"),
    ("implementation/file_permission", "os/filesystem/file_permission"),
    ("implementation/memory_configurations", "os/memory/memory_configurations"),
    ("implementation/crc", "os/libs/crc"),
    (
        "implementation/kernel_modules_vs_shared_libraries",
        "os/binfmt/kernel_modules_vs_shared_libraries",
    ),
    ("implementation/tls", "os/scheduling/tls"),
    ("implementation/user_identity", "os/scheduling/user_identity"),
    ("implementation/bottomhalf_interrupt", "os/interrupts/bottomhalf_interrupt"),
    ("implementation/interrupt_controls", "os/interrupts/interrupt_controls"),
    ("implementation/critical_sections", "os/interrupts/critical_sections"),
    ("implementation/tickless_os", "os/time/tickless_os"),
    ("implementation/short_time_delays", "os/time/short_time_delays"),
    (
        "implementation/oneshot_timers_and_cpu_load",
        "os/time/oneshot_timers_and_cpu_load",
    ),
    ("implementation/signal_handlers", "os/ipc/signal_handlers"),
    ("reference/os/addrenv", "os/memory/addrenv"),
    ("reference/os/iob", "os/memory/iob"),
    ("reference/os/events", "os/ipc/events"),
    ("reference/os/mutex", "os/ipc/mutex"),
    ("reference/os/sleep", "os/time/sleep"),
    ("reference/os/time_clock", "os/time/time_clock"),
    ("reference/os/wqueue", "os/scheduling/wqueue"),
    ("reference/os/newreno", "os/networking/newreno"),
    ("reference/os/led", "os/drivers/character/leds/index"),
    ("reference/os/arch", "os/arch/arch_api"),
    ("reference/os/board", "os/arch/board_api"),
    ("reference/os/notifier", "os/notifier"),
    ("reference/os/app_vs_os", "os/app_vs_os"),
    ("reference/os/nuttx", "os/nuttx"),
    ("reference/os/conventions", "os/conventions"),
]


# Pages that live in one of the new directories above but never lived in
# the old one: they came from implementation/ or reference/os/ (and have
# their own line in _OS_PAGES), or they are new.  Without this list the
# expansion below would add a redirect for an old URL that never existed.
_NOT_IN_OLD_TREE = {
    "components/arch/arch_api",
    "components/arch/board_api",
    "components/drivers/device_drivers",
    "components/drivers/device_nodes",
    "components/drivers/drivers_design",
    "components/drivers/ioctl",
    "components/drivers/special/power/power_management",
    "components/drivers/special/syslog_design",
    "components/drivers/usb",
    "components/filesystem/chroot",
    "components/filesystem/file_descriptors",
    "components/filesystem/file_permission",
    "components/libs/crc",
    "components/libs/libbuiltin",
    "components/mm/addrenv",
    "components/mm/iob",
    "components/mm/memory_configurations",
    "components/mm/paging",
    "components/net/newreno",
}


def _moved_trees(pairs):
    """Expand whole-directory moves into one redirect per page.

    Built from the pages that are there now, so a page added to a section
    later does not need a line here.
    """
    here = pathlib.Path(__file__).parent
    out = []
    for old_dir, new_dir in pairs:
        for page in sorted((here / new_dir).rglob("*.rst")):
            name = page.relative_to(here / new_dir).with_suffix("").as_posix()
            if f"{old_dir}/{name}" in _NOT_IN_OLD_TREE:
                continue
            out.append((f"{old_dir}/{name}", f"{new_dir}/{name}"))
    return out


# guides/ split by subject.  60 files in one directory, in no order anybody
# could use; each one now sits under the thing it is about.
_GUIDES = [
    (
        "guides/armv7m_runtimestackcheck",
        "guides/chip-specific/armv7m_runtimestackcheck",
    ),
    ("guides/automounter", "guides/filesystem/automounter"),
    (
        "guides/building_nuttx_with_app_out_of_src_tree",
        "guides/build/building_nuttx_with_app_out_of_src_tree",
    ),
    ("guides/building_uclibcpp", "guides/build/building_uclibcpp"),
    (
        "guides/changing_systemclockconfig",
        "guides/chip-specific/changing_systemclockconfig",
    ),
    ("guides/cpp_cmake", "guides/build/cpp_cmake"),
    ("guides/custom_app_directories", "guides/build/custom_app_directories"),
    ("guides/customapps", "guides/build/customapps"),
    ("guides/customboards", "guides/porting/customboards"),
    ("guides/devicetree", "guides/drivers/devicetree"),
    ("guides/drivers", "guides/drivers/drivers"),
    ("guides/etcromfs", "guides/filesystem/etcromfs"),
    ("guides/fork_vfork_migration", "guides/concurrency/fork_vfork_migration"),
    ("guides/fortify", "guides/security/fortify"),
    ("guides/fully_linked_elf", "guides/programs/fully_linked_elf"),
    ("guides/include_files_board_h", "guides/porting/include_files_board_h"),
    ("guides/integrate_newlib", "guides/build/integrate_newlib"),
    ("guides/ipv6", "guides/networking/ipv6"),
    (
        "guides/kernel_threads_with_custom_stacks",
        "guides/concurrency/kernel_threads_with_custom_stacks",
    ),
    ("guides/logging_rambuffer", "guides/drivers/logging_rambuffer"),
    ("guides/lwl", "guides/drivers/lwl"),
    ("guides/multiple_nsh_sessions", "guides/nsh/multiple_nsh_sessions"),
    ("guides/nestedinterrupts", "guides/concurrency/nestedinterrupts"),
    ("guides/nfs", "guides/networking/nfs"),
    ("guides/nix_flake", "guides/build/nix_flake"),
    (
        "guides/nsh_network_link_management",
        "guides/networking/nsh_network_link_management",
    ),
    ("guides/ofloader", "guides/drivers/ofloader"),
    ("guides/optee", "guides/security/optee"),
    ("guides/partially_linked_elf", "guides/programs/partially_linked_elf"),
    ("guides/platform_directories", "guides/build/platform_directories"),
    ("guides/port", "guides/porting/port"),
    ("guides/port_bootsequence", "guides/porting/port_bootsequence"),
    ("guides/port_drivers_to_stm32f7", "guides/chip-specific/port_drivers_to_stm32f7"),
    (
        "guides/port_relatedkernelconfigrations",
        "guides/porting/port_relatedkernelconfigrations",
    ),
    ("guides/protected_build", "guides/programs/protected_build"),
    ("guides/pysimcoder", "guides/languages/pysimcoder"),
    ("guides/qemu_tips", "guides/simulation/qemu_tips"),
    ("guides/ram_rom_disks", "guides/filesystem/ram_rom_disks"),
    ("guides/reading_can_msgs", "guides/drivers/reading_can_msgs"),
    ("guides/remove_device_drivers_nsh", "guides/nsh/remove_device_drivers_nsh"),
    ("guides/renode", "guides/simulation/renode"),
    ("guides/rndis", "guides/drivers/rndis"),
    ("guides/rust", "guides/languages/rust"),
    ("guides/semihosting", "guides/chip-specific/semihosting"),
    (
        "guides/signal_events_interrupt_handlers",
        "guides/concurrency/signal_events_interrupt_handlers",
    ),
    (
        "guides/signaling_sem_priority_inheritance",
        "guides/concurrency/signaling_sem_priority_inheritance",
    ),
    ("guides/simulator", "guides/simulation/simulator"),
    ("guides/smaller_vector_tables", "guides/chip-specific/smaller_vector_tables"),
    (
        "guides/specialstuff_in_nuttxheaderfiles",
        "guides/porting/specialstuff_in_nuttxheaderfiles",
    ),
    ("guides/stm32_ports", "guides/chip-specific/stm32_ports"),
    ("guides/stm32ccm", "guides/chip-specific/stm32ccm"),
    ("guides/stm32nullpointer", "guides/chip-specific/stm32nullpointer"),
    ("guides/testingtcpip", "guides/networking/testingtcpip"),
    ("guides/thread_local_storage", "guides/concurrency/thread_local_storage"),
    (
        "guides/updating_release_system_elf",
        "guides/programs/updating_release_system_elf",
    ),
    ("guides/usbtrace", "guides/drivers/usbtrace"),
    ("guides/usingkernelthreads", "guides/concurrency/usingkernelthreads"),
    (
        "guides/versioning_and_task_names",
        "guides/concurrency/versioning_and_task_names",
    ),
    ("guides/zerolatencyinterrupts", "guides/concurrency/zerolatencyinterrupts"),
    (
        "guides/porting-case-studies/bcm2711-rpi4b",
        "guides/porting/case-studies/bcm2711-rpi4b",
    ),
    (
        "guides/porting-case-studies/port_arm_cm4",
        "guides/porting/case-studies/port_arm_cm4",
    ),
]

redirects = _moved(
    _PLATFORM_ALIGNMENT
    + _TAG_VOCABULARY
    + _CHIP_TO_PART
    + _DROPPED_ROLLUPS
    + _DROPPED_BOARD_VENDORS
    + _DROPPED_FACTS
    + _SOURCE_TREE_TRUTH
    + _X86_64_BOARD
    + _OS_SCHEDULING
    + _OS_PAGES
    + _moved_trees(_OS_TREES)
    + _GUIDES
)
