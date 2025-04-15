#!/usr/bin/env python3
# tools/splashscreen_converter.py
#
# SPDX-License-Identifier: Apache-2.0
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

"""This script converts from any image type supported by
Python imaging library to the RLE-encoded format used by
the framebuffer splashscreen feature.
"""

from PIL import Image


def get_palette(img, maxcolors=256):
    """Returns a list of colours. If there are too many colours in the image,
    the least used are removed.
    """

    colors = img.getcolors(65536)
    colors.sort(key=lambda c: -c[0])
    return [c[1] for c in colors[:maxcolors]]


def write_palette(outfile, palette, type):
    """Write the palette to the output file."""

    if type == "ARGB":
        outfile.write("#ifdef CONFIG_VIDEO_FB_SPLASHSCREEN_BPP32\n")
        outfile.write("static const fb_pixel_t palette[] =\n")
        outfile.write("{\n")

        for i in range(0, len(palette), 4):
            for r, g, b, a in palette[i : i + 4]:
                outfile.write("  MKRGB(%d, %d, %d, %d),\n" % (a, r, g, b))

        outfile.write("};\n")
        outfile.write("#endif\n\n")

    elif type == "RGB":
        outfile.write("#if defined(CONFIG_VIDEO_FB_SPLASHSCREEN_BPP24) || \\\n")
        outfile.write("    defined(CONFIG_VIDEO_FB_SPLASHSCREEN_BPP16) || \\\n")
        outfile.write("    defined(CONFIG_VIDEO_FB_SPLASHSCREEN_BPP8)\n")
        outfile.write("static const fb_pixel_t palette[] =\n")
        outfile.write("{\n")

        for i in range(0, len(palette), 4):
            for r, g, b in palette[i : i + 4]:
                outfile.write("  MKRGB(%d, %d, %d),\n" % (r, g, b))

        outfile.write("};\n")
        outfile.write("#endif\n\n")

    else:
        outfile.write("#if defined(CONFIG_VIDEO_FB_SPLASHSCREEN_GREY) || \\\n")
        outfile.write("    defined(CONFIG_VIDEO_FB_SPLASHSCREEN_MONO)\n")
        outfile.write("static const fb_pixel_t palette[] =\n{\n};\n")
        outfile.write("#endif\n\n")


def quantize(color, palette):
    """Return the color index to closest match in the palette."""
    try:
        return palette.index(color)
    except ValueError:
        # No exact match, search for the closest
        def distance(color2):
            return sum([(a - b) ** 2 for a, b in zip(color, color2)])

    return palette.index(min(palette, key=distance))


def encode_row(img, palette, y, type):
    """RLE-encode one row of image data."""

    color = None
    entries = []
    repeats = 0

    for x in range(0, img.width):
        if type == "BPP32" or type == "BPP24" or type == "BPP16":
            c = quantize(img.getpixel((x, y)), palette)
        else:
            c = img.getpixel((x, y))
        if c == color and repeats < 255:
            repeats += 1
        else:
            if color is not None:
                entries.append((repeats, color))

            repeats = 1
            color = c

    if color is not None:
        entries.append((repeats, color))

    return entries


def write_image(outfile, img, palette, suffix):
    """Write the image contents to the output file."""

    if suffix == "BPP24" or suffix == "BPP16":
        outfile.write("#if defined(CONFIG_VIDEO_FB_SPLASHSCREEN_BPP24) || \\")
        outfile.write("\n    defined(CONFIG_VIDEO_FB_SPLASHSCREEN_BPP16)\n")
    else:
        outfile.write("#ifdef CONFIG_VIDEO_FB_SPLASHSCREEN_%s\n" % suffix)
    outfile.write("static const struct splscr_bitmap_s bitmap[] =\n")
    outfile.write("{")
    for y in range(0, img.height):
        entries = encode_row(img, palette, y, suffix)
        for r, c in entries:
            outfile.write("\n")
            row = "  {%d, %d}," % (r, c)
            outfile.write(row)

        outfile.write(("/* End of row %3d */" % (y + 1)).rjust(78 - len(row), " "))

    outfile.write("\n};\n")
    outfile.write("#endif /* CONFIG_VIDEO_FB_SPLASHSCREEN_%s */\n\n" % suffix)


def write_descriptor(outfile, name):
    outfile.write("const struct palette_bitmap_s g_%s =\n" % name)
    outfile.write("{\n")
    lw = len(str(img.width))
    lh = len(str(img.height))
    outfile.write("  %d," % img.width)
    outfile.write(
        ("/* width in pixels                                   */\n").rjust(
            76 - lw, " "
        )
    )
    outfile.write("  %d," % img.height)
    outfile.write(
        ("/* height in pixels                                  */\n").rjust(
            76 - lh, " "
        )
    )
    outfile.write(
        (
            "  palette,             /* Colour palette                                    */\n"
        ).rjust(76, " ")
    )
    outfile.write(
        (
            "  bitmap,              /* Pointer to the start of the RLE data              */\n"
        ).rjust(76, " ")
    )
    outfile.write("};\n")


if __name__ == "__main__":
    import os.path
    import sys

    if len(sys.argv) < 3:
        print("Usage: splashscreen_converter.py image.xxx output_directory out.c")
        print(
            "\t- image.xxx\t\tis the image file (e.g. logo.png) in a format supported by PIL"
        )
        print("\t- output_directory\tis where the output file will be saved.")
        print("\t- out.c\t\t\tis the name of the output file.")
        print("\t\t\t\t- If out.c is not specified it will default to fb_splash.c,")
        print("\t\t\t\t  which is the name required for custom splashscreens")
        sys.exit(1)
    elif len(sys.argv) == 3:
        path = sys.argv[2]
        filename = "fb_splash.c"
    else:
        path = os.path.relpath(sys.argv[2])
        filename = sys.argv[3]
    img = Image.open(sys.argv[1]).convert("RGBA")
    file = os.path.realpath(path + "/" + filename)
    outfile = open(file, "w")
    palette_argb = get_palette(img)
    palette_rgb = get_palette(img.convert("RGB"))
    outfile.write(
        """/****************************************************************************
 * %(file)s
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Script-generated framebuffer splashscreen bitmap file.
 * Generated from %(src)s
 * by splashscreen_converter.py
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/video/fb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

"""
        % {"file": path + "/" + filename, "src": os.path.relpath(sys.argv[1])}
    )

    name = "splscr"

    write_palette(outfile, palette_argb, "ARGB")
    write_palette(outfile, palette_rgb, "RGB")
    write_palette(outfile, None, None)
    write_image(outfile, img, palette_argb, "BPP32")
    write_image(outfile, img.convert("RGB"), palette_rgb, "BPP24")
    write_image(outfile, img.convert("L"), None, "GREY")
    write_image(outfile, img.convert("1"), None, "MONO")
    write_descriptor(outfile, name)
    outfile.write(
        """
/****************************************************************************
 * Private Functions
 ****************************************************************************/
"""
    )
    print("Created %s from %s" % (file, sys.argv[1]))
