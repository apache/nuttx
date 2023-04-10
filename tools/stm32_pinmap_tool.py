#!/usr/bin/env python3
############################################################################
# tools/stm32_pinmap_tool.py
#
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
#
############################################################################

# for python2.7 compatibility
from __future__ import print_function

import argparse
import os
import re
import sys
from argparse import RawTextHelpFormatter
from glob import glob

suffix = "_0"
remaps_re = re.compile(r"(.*REMAP.*)=y")
ip_block_re = re.compile(r"CONFIG_STM32[A-Z0-9]*_([A-Z0-9]+[0-9]*)=")
stm32f1_re = re.compile(r"stm32f10[0-9][a-z]*_pinmap")
speed_re = re.compile(r"(GPIO_(?:SPEED|MODE)_[zA-Z0-9]+)")
port_re = re.compile(r"GPIO_PORT([A-Z])\|")
pin_re = re.compile(r"GPIO_PIN(\d+)")
define_re = re.compile(r"#\s*define\s+(GPIO.*)\s+(GPIO.*?)\s+")


class GPIODef:
    def __init__(self, original_name, name, description):
        self.original_name = original_name
        self.name = name
        self.block = name.split("_")[1]
        self.speed = None
        s = speed_re.search(description)
        if s:
            self.speed = s.group(1)
        s = port_re.search(description)
        if s:
            self.port = s.group(1)
        s = pin_re.search(description)
        if s:
            self.pin = s.group(1)

    def __str__(self):
        fmt = "#define {0: <20} {1} /* P{2} */"
        if self.speed:
            if "MODE" in self.speed:
                if "MHz" in self.speed:
                    # F1 has mode, MHz is output, we must adjust the speed
                    fmt = "#define {0: <20} GPIO_ADJUST_MODE({1}, {3}) /* P{2} */ "
            else:
                # All others had a OSPEDD reg so we just set it
                fmt = "#define {0: <20} ({1} | {3}) /* P{2} */ "

        return fmt.format(
            self.original_name,
            self.name,
            self.port + self.pin,
            self.speed,
        )

    def __repr__(self):
        return f"<GPIODef block:{self.block} \
                 original_name:{self.original_name} \
                 name:{self.name} port:{self.port} \
                 pin:{self.pin} speed:{self.speed}>"


# Detect python version
if sys.version_info[0] < 3:
    runningPython3 = False
else:
    runningPython3 = True


def parse_args():
    # Parse commandline arguments
    parser = argparse.ArgumentParser(
        formatter_class=RawTextHelpFormatter,
        description="""stm32_pinmap_tool.py

   This tool is used to migrate legacy stm32 pinmap files that
   had included pin speed (slew rate control) in pinmap pin definitions

   These speeds should have never been part of the arch defines as these
   are layout and board dependent. Therefore, the complete definition
   should be a composition of the pinmap defines and speed, and defined in
   board.h

   Furthermore, pinmaps did not suffix pins that had only one ALT
   appearance on a GPIO. Therefore there was no way to change the speed
   or any other pins attribute i.e. Pullup Pulldown, Push pull. Open Drain etc.

    The tool has a conversion mode and a report mode.

 Conversion mode tool use:

    Run the tool to do the conversion:
 i.e tools/stm32_pinmap_tool.py
       --pinmap  arch/arm/src/stm32h7/hardware/stm32h7x3xx_pinmap.h
       --legacy > arch/arm/src/stm32h7/hardware/stm32h7x3xx_pinmap-new.h

   -- pinmap - the file to convert
   --legacy will make a copy of the pinmap. Properly updating the file with
     xxxx/xxxxxxx_legacy to the title block,
     and adding _LEGACY to the #ifdef, #define and endif comment of the inclusion guard.

 Conversion mode follow up edits:
   1.  diff and verify the original pinmap and the pinmap-new.h are as expected.
       delete original pinmap
       rename pinmap-new.h to the original pinmap name.
   2. Edit the top level pinmap (i.e. arch/arm/src/stm32x/stm32x_pinmap.h) file and
      add a CONFIG_STM32xx_USE_LEGACY_PINMAP section
      that includes the legacy pinmap files.

   For example
          if defined(CONFIG_STM32H7_USE_LEGACY_PINMAP)
            if defined(CONFIG_STM32H7_STM32H7X3XX)
              include "hardware/stm32h7x3xx_pinmap_legacy.h"
            elif defined(CONFIG_STM32H7_STM32H7B3XX)
              include "hardware/stm32h7x3xx_pinmap_legacy.h"
            elif defined(CONFIG_STM32H7_STM32H7X7XX)
              include "hardware/stm32h7x3xx_pinmap_legacy.h"
            else
              error "Unsupported STM32 H7 Pin map"
            endif
          else
            if defined(CONFIG_STM32H7_STM32H7X3XX)
              include "hardware/stm32h7x3xx_pinmap.h"
            elif defined(CONFIG_STM32H7_STM32H7B3XX)
              include "hardware/stm32h7x3xx_pinmap.h"
            elif defined(CONFIG_STM32H7_STM32H7X7XX)
              include "hardware/stm32h7x3xx_pinmap.h"
            else
              error "Unsupported STM32 H7 Pin map"
            endif
          endif

   3. Add a STM32Hx_USE_LEGACY_PINMAP to the Kconfig defaulted to y

   For example

       config STM32H7_USE_LEGACY_PINMAP
           bool "Use the legacy pinmap with GPIO_SPEED_xxx included."
           default y
           ---help---
               In the past, pinmap files included GPIO_SPEED_xxxMhz. These speed
               settings should have come from the board.h as it describes the wiring
               of the SoC to the board. The speed is really slew rate control and
               therefore is related to the layout and can only be properly set
               in board.h.

               STM32H7_USE_LEGACY_PINMAP is provided, to allow lazy migration to
               using pinmaps without speeds. The work required to do this can be aided
               by running tools/stm32_pinmap_tool.py. The tools will take a board.h
               file and a legacy pinmap and output the required changes that one needs
               to make to a board.h file.

               Eventually, STM32H7_USE_LEGACY_PINMAP will be deprecated and the legacy
               pinmaps removed from NuttX. Any new boards added should set
               STM32H7_USE_LEGACY_PINMAP=n and fully define the pins in board.h
   4. Add a warning to the xxx_gpio.c file

   For example

       #if defined(CONFIG_STM32_USE_LEGACY_PINMAP)
       #  pragma message "CONFIG_STM32_USE_LEGACY_PINMAP will be deprecated migrate board.h see tools/stm32_pinmap_tool.py"
       #endif

 Report mode tool use:

    Run the tool to aid in migrating a board.h

   tools/stm32_pinmap_tool.py --pinmap arch/arm/src/stm32h7/hardware/stm32h7x3xx_pinmap_legacy.h
   --report <fullpath>/include/board.h

   it will output 2 sections that should be used to update the board.h.
   board.h defines that need to have speeds added.
   board.h defines that will need to be added:
    """,
    )

    parser.add_argument(
        "--pinmap",
        action="store",
        help="""pin map file to convert (changes are printed on stdout) or
                Legacy file pin map file named <filename>_legacy.<ext> to report board.h changes""",
    )
    parser.add_argument(
        "--report",
        default=False,
        action="store",
        help="Generate change set for a board",
    )
    parser.add_argument(
        "--legacy",
        default=False,
        action="store_true",
        help="If one does not exist, create a copy of the original pin map named <filename>_legacy.<ext>",
    )
    args = parser.parse_args()
    return args


def create_legacy(source):
    legacy = source.replace(".h", "_legacy.h")
    sourceshort = source[source.find("arch") :]
    legacyshort = legacy[legacy.find("arch") :]
    srctag = "__" + sourceshort.upper().replace("/", "_")
    destag = "__" + legacyshort.upper().replace("/", "_").replace(".", "_")
    if not os.path.isfile(legacy):
        fout = open(legacy, "w")
        fin = open(source, "r")

        for line in fin:
            out = re.sub(sourceshort, legacyshort, line)
            out = re.sub(srctag, destag, out)
            fout.write(out)
        fout.close()
        fin.close()


def read_defconfigs(boardfile_path):
    configs_lines = []
    defconfigs_files = []

    for dir, _, _ in os.walk(boardfile_path[: boardfile_path.find("include/board.h")]):
        defconfigs_files.extend(glob(os.path.join(dir, "defconfig")))

    for file in defconfigs_files:
        defconfigfile = open(file, "r")
        configs_lines.extend(defconfigfile.readlines())
        defconfigfile.close()
    return configs_lines


def build_ip_remap_list(boardfile_path):
    ip_blocks = []
    ip_remaps = []
    configs_lines = read_defconfigs(boardfile_path)
    configs_lines = sorted(set(configs_lines))

    for line in configs_lines:
        s = ip_block_re.search(line)
        if s:
            ip_blocks.extend([s.group(1)])
        else:
            s = remaps_re.search(line)
            if s:
                ip_remaps.extend([s.group(1)])
    return [ip_blocks, ip_remaps]


def read_board_h(boardfile_path):
    boardfile = open(boardfile_path, "r")
    lines = boardfile.readlines()
    boardfile.close()
    return lines


def formated_print(lines):
    maxlen = 0
    for line in lines:
        linelen = line.find("/*")
        if linelen > maxlen:
            maxlen = linelen

    for line in lines:
        linelen = line.find("/*")
        if linelen > 1 and linelen < maxlen:
            nl = line[:linelen] + " " * (maxlen - linelen) + line[linelen:]
            line = nl
        print(line)


def report(boardfile_path, boards_ip_blocks, changelog, changelog_like):
    output = [
        "",
    ]
    output.extend(
        [
            """
There were 3 issues with the Legacy pinmaps.

 1. The legacy version of the pin defines included speed settings. (These are
    in reality, slew rates).

 2. Legacy pinmaps erroneously added speeds on pins that are only used
    as an inputs (i.e UART4_RX). These speeds can be removed from the board.h
    defines.

 3. Also the legacy version of the pin defines did not have a suffix on all
    pins and therefore all pins could not have the attributes set or changed
    by board.h

The new pinmaps correct these issues:

 Pin that had an explicit (GPIO_SPEED|MODE)_xxxMHz are removed or set to
 the lowest speed.

 If the pin had only one choice previously (un-suffixed) the pin name now
 contains _0 as the suffix.

 N.B. The correct speed setting for a given pin is very dependent on the
 layout of the circuit board and load presented to the SoC on that pin.

 The speeds listed below are from the Legacy pinmaps and are provided ONLY
 to insure these changes do not break existing systems that are relying on
 the legacy speed settings.

 It highly recommended that the speed setting for each pin be verified for
 overshoot and undershoot on real hardware and adjusted in the board,h
 appropriately.


board.h defines that need to have speeds added.

"""
        ]
    )

    boards_blocks = []
    Lines = read_board_h(boardfile_path)
    for line in Lines:
        s = define_re.search(line)
        if s:
            # #define GPIO_SD_CK   GPIO_SD_CK_1  /* PD6  FC_PD6_SD_CK  */
            define = s.group(1)
            original_name = s.group(2)
            change = changelog.get(original_name)
            if change:
                pindef = GPIODef(define, original_name, line)
                if pindef.block not in boards_blocks:
                    boards_blocks.append(pindef.block)
                    output.extend([f"\n/* {pindef.block} */\n"])
                output.extend([str(changelog[original_name])])
    if len(boards_blocks) == 0:
        output.extend(
            [
                """
   No pins are defined in board.h to change speeds on (most likely an stm32f1")
   We will define all the pins used next...
            """
            ]
        )

    formated_print(output)
    output = []

    output.extend(
        [
            """

 Pin that had only one choice previously (un-suffixed) pins will need to be
 defined in board.h to map the un-suffixed) pin name used in the drives to
 the _0 suffixed ones.

 Pins that did not have an explicit (GPIO_SPEED|MODE)_xxxMHz specified are
 listed with the pin name containing the new suffix.


board.h defines that may need to be added if the pins are used on the board:


"""
        ]
    )

    for block in boards_ip_blocks:
        change = changelog_like.get(block)
        if change:
            block_title = f"\n/* {block} */\n"
            for gpio in change:
                if re.search(r"_\d+$", gpio.original_name) is None:
                    if block_title:
                        output.extend([block_title])
                        block_title = None
                    output.extend([str(gpio)])

    formated_print(output)


def formatcols(list, cols):
    lines = ("\t".join(list[i : i + cols]) for i in range(0, len(list), cols))
    return "\n".join(lines)


def parse_conditional(lines, conditions):
    defines = []

    def_remap_re = re.compile(r"\s*defined\s*\((.*REMAP.*)\)")
    def_else_re = re.compile(r"#\s*else")
    def_endif_re = re.compile(r"#\s*endif")

    active_define = None
    output = True
    once = False

    for line in lines:
        # process #[el]if define(...REMAP)
        s = def_remap_re.search(line)
        if s:
            once = True
            define = s.group(1)
            if define in conditions:
                active_define = define
                output = True
            else:
                output = False
        else:
            # process #endif
            s = def_endif_re.search(line)
            if s:
                active_define = None
                output = True
            else:
                # process #elese
                s = def_else_re.search(line)
                if s:
                    once = True
                    # the if or elif was taken do not output the else
                    if active_define:
                        output = False
                    else:
                        output = output ^ True

        if once or output:
            once = False
            defines.extend([line])
    return defines


def formmatter(args):
    # if pinmap passed is a legacy pinmap. Just generate a report
    report_only = args.report is not False

    speed_not_mode = stm32f1_re.search(args.pinmap) is None

    if not report_only and args.legacy is True:
        create_legacy(args.pinmap)

    pinfile = open(args.pinmap, "r")
    Lines = pinfile.readlines()

    if report_only:
        boards_ip_blocks, remaps = build_ip_remap_list(args.report)
        print(
            f"\n\nBoard enabled Blocks:\n\n{formatcols(sorted(boards_ip_blocks), 8)}\n\n"
        )
        if (
            "ADC1" in boards_ip_blocks
            or "ADC2" in boards_ip_blocks
            or "ADC3" in boards_ip_blocks
        ):
            boards_ip_blocks.extend(["ADC12"])
            boards_ip_blocks.extend(["ADC123"])
            boards_ip_blocks = sorted(boards_ip_blocks)
        # Filter out ifdefed by remap conditionals (F1)
        if len(remaps) > 0:
            Lines = parse_conditional(Lines, remaps)

    Pass = False
    inComment = False

    changelog = {}
    changelog_like = {}
    pass_list = [r"#\s*if", r"#\s*else", r"#\s*end", r"#\s*include", r"#\s*undef"]
    pass_list_re = re.compile("|".join(pass_list))

    for line in Lines:
        if len(line.strip()) == 0:
            Pass = True
        if pass_list_re.search(line):
            Pass = True
        if "#define" in line and "GPIO" not in line:
            Pass = True
        if "defined(" in line:
            Pass = True
        if "/*" in line:
            inComment = True
            Pass = True
        if "*/" in line:
            inComment = False
            Pass = True
        if Pass or inComment:
            Pass = False
            if not report_only:
                print(line.rstrip(), end="")
        else:
            changed = False
            # split the line on spaces
            pieces = line.split()
            # deal with white space in the #  define for nested defines
            sel = 0
            # Does it have white space then use next set?
            if pieces[0] == "#":
                sel = 1
            original_name = pieces[sel + 1]
            gpiocgf = pieces[sel + 2]
            new_name = original_name
            if re.search(r"_\d+$", original_name) is None:
                # Add suffix
                pad = ""
                sel = line.find(original_name) + len(original_name)
                if line[sel + len(suffix)] == "(":
                    pad = " "
                if line[sel + len(suffix)] == "G":
                    pad = "  ("
                nl = line[:sel] + suffix + pad + line[sel + len(suffix) :]
                new_name = original_name + suffix
                changed = True
            else:
                nl = line
            # Remove the speed or chege the Mode
            if speed_not_mode:
                ol = re.sub(r"\s*GPIO_SPEED_[zA-Z0-9]+\s*\|", "", nl)
            else:
                ol = re.sub(
                    r"(\s*)GPIO_MODE_[0-9]+MHz(\s*\|)", r"\g<1>GPIO_MODE_2MHz\g<2>", nl
                )

            changed = changed or ol != nl
            if not report_only:
                print(ol.strip(), end="")
            if args.report and changed:
                changelog[original_name] = pindef = GPIODef(
                    original_name, new_name, gpiocgf
                )

                # create changes by block if enabled
                if pindef.block in boards_ip_blocks:
                    # Is block in already?
                    if pindef.block in changelog_like:
                        # do not duplicate it
                        if pindef not in changelog_like[pindef.block]:
                            changelog_like[pindef.block].append(pindef)
                    else:
                        changelog_like[pindef.block] = [pindef]

        if not report_only:
            print("")
    if args.report:
        report(args.report, boards_ip_blocks, changelog, changelog_like)


def main():
    # Python2 is EOL
    if not runningPython3:
        raise RuntimeError(
            "Python 2 is not supported. Please try again using Python 3."
        )
    args = parse_args()
    formmatter(args)


if __name__ == "__main__":
    main()
