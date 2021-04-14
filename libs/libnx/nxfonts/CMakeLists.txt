############################################################################
# libs/libnx/nxfonts/Make.defs
#
#   Copyright (C) 2008, 2013, 2017 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Expose NXFONT interfaces to applications

if("${CONFIG_NXFONTS}" STREQUAL "y")

	set(CSRCS)
	
	
############################################################################
# libs/libnx/nxfonts/Makefile.sources
#
#   Copyright (C) 2008, 2013, 2017 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

ifdef NXFONTS_BITSPERPIXEL
	if("${NXFONTS_BITSPERPIXEL),1)
		NXFONTS_SUFFIX := _1bpp
		GEN_CSRC := nxfonts_convert_1bpp.c
	endif
	if("${NXFONTS_BITSPERPIXEL),2)
		NXFONTS_SUFFIX := _2bpp
		GEN_CSRC := nxfonts_convert_2bpp.c
	endif
	if("${NXFONTS_BITSPERPIXEL),4)
		NXFONTS_SUFFIX := _4bpp
		GEN_CSRC := nxfonts_convert_4bpp.c
	endif
	if("${NXFONTS_BITSPERPIXEL),8)
		NXFONTS_SUFFIX := _8bpp
		GEN_CSRC := nxfonts_convert_8bpp.c
	endif
	if("${NXFONTS_BITSPERPIXEL),16)
		NXFONTS_SUFFIX := _16bpp
		GEN_CSRC := nxfonts_convert_16bpp.c
	endif
	if("${NXFONTS_BITSPERPIXEL),24)
		NXFONTS_SUFFIX := _24bpp
		GEN_CSRC := nxfonts_convert_24bpp.c
	endif
	if("${NXFONTS_BITSPERPIXEL),32)
		NXFONTS_SUFFIX := _32bpp
		GEN_CSRC := nxfonts_convert_32bpp.c
	endif
	
	#DEPENDENCY := nxfonts_convert.c
endif

ifdef NXFONTS_FONTID

# MONO

if("${NXFONTS_FONTID}" STREQUAL "18")
	NXFONTS_PREFIX := g_mono5x8_
	GEN_CSRC = nxfonts_bitmaps_mono5x8.c
endif()

# SANS

if("${NXFONTS_FONTID}" STREQUAL "14)
NXFONTS_PREFIX := g_sans17x22_
GEN_CSRC = nxfonts_bitmaps_sans17x22.c
endif

if("${NXFONTS_FONTID}" STREQUAL "15)
NXFONTS_PREFIX := g_sans20x26_
GEN_CSRC = nxfonts_bitmaps_sans20x26.c
endif

if("${NXFONTS_FONTID}" STREQUAL "1)
NXFONTS_PREFIX := g_sans23x27_
GEN_CSRC = nxfonts_bitmaps_sans23x27.c
endif

if("${NXFONTS_FONTID}" STREQUAL "2)
NXFONTS_PREFIX := g_sans22x29_
GEN_CSRC = nxfonts_bitmaps_sans22x29.c
endif

if("${NXFONTS_FONTID}" STREQUAL "3)
NXFONTS_PREFIX := g_sans28x37_
GEN_CSRC = nxfonts_bitmaps_sans28x37.c
endif

if("${NXFONTS_FONTID}" STREQUAL "4)
NXFONTS_PREFIX := g_sans39x48_
GEN_CSRC = nxfonts_bitmaps_sans39x48.c
endif

# SANS-BOLD

if("${NXFONTS_FONTID}" STREQUAL "16)
NXFONTS_PREFIX := g_sans17x23b_
GEN_CSRC = nxfonts_bitmaps_sans17x23b.c
endif

if("${NXFONTS_FONTID}" STREQUAL "17)
NXFONTS_PREFIX := g_sans20x27b_
GEN_CSRC = nxfonts_bitmaps_sans20x27b.c
endif

if("${NXFONTS_FONTID}" STREQUAL "5)
NXFONTS_PREFIX := g_sans22x29b_
GEN_CSRC = nxfonts_bitmaps_sans22x29b.c
endif

if("${NXFONTS_FONTID}" STREQUAL "6)
NXFONTS_PREFIX := g_sans28x37b_
GEN_CSRC = nxfonts_bitmaps_sans28x37b.c
endif

if("${NXFONTS_FONTID}" STREQUAL "7)
NXFONTS_PREFIX := g_sans40x49b_
GEN_CSRC = nxfonts_bitmaps_sans40x49b.c
endif

# SERIF

if("${NXFONTS_FONTID}" STREQUAL "8)
NXFONTS_PREFIX := g_serif22x29_
GEN_CSRC = nxfonts_bitmaps_serif22x29.c
endif

if("${NXFONTS_FONTID}" STREQUAL "9)
NXFONTS_PREFIX := g_serif29x37_
GEN_CSRC = nxfonts_bitmaps_serif29x37.c
endif

if("${NXFONTS_FONTID}" STREQUAL "10)
NXFONTS_PREFIX := g_serif38x48_
GEN_CSRC = nxfonts_bitmaps_serif38x48.c
endif

# SERIF-BOLD

if("${NXFONTS_FONTID}" STREQUAL "11)
NXFONTS_PREFIX := g_serif22x28b_
GEN_CSRC = nxfonts_bitmaps_serif22x28b.c
endif

if("${NXFONTS_FONTID}" STREQUAL "12)
NXFONTS_PREFIX := g_serif27x38b_
GEN_CSRC = nxfonts_bitmaps_serif27x38b.c
endif

if("${NXFONTS_FONTID}" STREQUAL "13)
NXFONTS_PREFIX := g_serif38x49b_
GEN_CSRC = nxfonts_bitmaps_serif38x49b.c
endif

# Pixel fonts

if("${NXFONTS_FONTID}" STREQUAL "19)
	NXFONTS_PREFIX := g_pixel_unicode_
	GEN_CSRC = nxfonts_bitmaps_pixel-unicode.c
endif

if("${NXFONTS_FONTID}" STREQUAL "20)
	NXFONTS_PREFIX := g_pixel_lcd_machine_
	GEN_CSRC = nxfonts_bitmaps_pixel-lcd-machine.c
endif

# X11-misc-fixed fonts

if("${NXFONTS_FONTID}" STREQUAL "21)
	NXFONTS_PREFIX := g_x11_misc_fixed_4x6_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-4x6.c
endif

if("${NXFONTS_FONTID}" STREQUAL "22)
	NXFONTS_PREFIX := g_x11_misc_fixed_5x7_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-5x7.c
endif

if("${NXFONTS_FONTID}" STREQUAL "23)
	NXFONTS_PREFIX := g_x11_misc_fixed_5x8_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-5x8.c
endif

if("${NXFONTS_FONTID}" STREQUAL "24)
	NXFONTS_PREFIX := g_x11_misc_fixed_6x9_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-6x9.c
endif

if("${NXFONTS_FONTID}" STREQUAL "25)
	NXFONTS_PREFIX := g_x11_misc_fixed_6x10_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-6x10.c
endif

if("${NXFONTS_FONTID}" STREQUAL "26)
	NXFONTS_PREFIX := g_x11_misc_fixed_6x12_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-6x12.c
endif

if("${NXFONTS_FONTID}" STREQUAL "27)
	NXFONTS_PREFIX := g_x11_misc_fixed_6x13_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-6x13.c
endif

if("${NXFONTS_FONTID}" STREQUAL "28)
	NXFONTS_PREFIX := g_x11_misc_fixed_6x13B_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-6x13B.c
endif

if("${NXFONTS_FONTID}" STREQUAL "29)
	NXFONTS_PREFIX := g_x11_misc_fixed_6x13O_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-6x13O.c
endif

if("${NXFONTS_FONTID}" STREQUAL "30)
	NXFONTS_PREFIX := g_x11_misc_fixed_7x13_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-7x13.c
endif

if("${NXFONTS_FONTID}" STREQUAL "31)
	NXFONTS_PREFIX := g_x11_misc_fixed_7x13B_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-7x13B.c
endif

if("${NXFONTS_FONTID}" STREQUAL "32)
	NXFONTS_PREFIX := g_x11_misc_fixed_7x13O_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-7x13O.c
endif

if("${NXFONTS_FONTID}" STREQUAL "33)
	NXFONTS_PREFIX := g_x11_misc_fixed_7x14_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-7x14.c
endif

if("${NXFONTS_FONTID}" STREQUAL "34)
	NXFONTS_PREFIX := g_x11_misc_fixed_7x14B_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-7x14B.c
endif

if("${NXFONTS_FONTID}" STREQUAL "35)
	NXFONTS_PREFIX := g_x11_misc_fixed_8x13_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-8x13.c
endif

if("${NXFONTS_FONTID}" STREQUAL "36)
	NXFONTS_PREFIX := g_x11_misc_fixed_8x13B_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-8x13B.c
endif

if("${NXFONTS_FONTID}" STREQUAL "37)
NXFONTS_PREFIX := g_x11_misc_fixed_8x13O_
GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-8x13O.c
endif

if("${NXFONTS_FONTID}" STREQUAL "38)
	NXFONTS_PREFIX := g_x11_misc_fixed_9x15_
	GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-9x15.c
endif

if("${NXFONTS_FONTID}" STREQUAL "39)
NXFONTS_PREFIX := g_x11_misc_fixed_9x15B_
GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-9x15B.c
endif

if("${NXFONTS_FONTID}" STREQUAL "40)
NXFONTS_PREFIX := g_x11_misc_fixed_9x18_
GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-9x18.c
endif

if("${NXFONTS_FONTID}" STREQUAL "41)
NXFONTS_PREFIX := g_x11_misc_fixed_9x18B_
GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-9x18B.c
endif

if("${NXFONTS_FONTID}" STREQUAL "42)
NXFONTS_PREFIX := g_x11_misc_fixed_10x20_
GEN_CSRC = nxfonts_bitmaps_x11-misc-fixed-10x20.c
endif

# Tom Thumb mono-space 4x6 font

if("${NXFONTS_FONTID}" STREQUAL "43)
	NXFONTS_PREFIX := g_tom_thumb_4x6_
	GEN_CSRC = nxfonts_tom-thumb-4x6.c
endif
	

	list(APPEND CSRCS nxfonts_getfont.c nxfonts_cache.c)
	list(APPEND CSRCS nxfonts_convert_1bpp.c nxfonts_convert_2bpp.c)
	list(APPEND CSRCS nxfonts_convert_4bpp.c nxfonts_convert_8bpp.c)
	list(APPEND CSRCS nxfonts_convert_16bpp.c nxfonts_convert_24bpp.c)
	list(APPEND CSRCS nxfonts_convert_32bpp.c)
	
	# Monospace fonts
	
	if("${CONFIG_NXFONT_MONO5X8}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_mono5x8.c)
	endif()
	
	# Sans serif fonts
	
	if("${CONFIG_NXFONT_SANS17X22}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans17x22.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS20X26}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans20x26.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS23X27}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans23x27.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS22X29}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans22x29.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS28X37}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans28x37.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS39X48}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans39x48.c)
	endif()
	
	# Sans serif bold fonts
	
	if("${CONFIG_NXFONT_SANS17X23B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans17x23b.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS20X27B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans20x27b.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS22X29B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans22x29b.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS28X37B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans28x37b.c)
	endif()
	
	if("${CONFIG_NXFONT_SANS40X49B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_sans40x49b.c)
	endif()
	
	# Serif fonts
	
	if("${CONFIG_NXFONT_SERIF22X29}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_serif22x29.c)
	endif()
	
	if("${CONFIG_NXFONT_SERIF29X37}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_serif29x37.c)
	endif()
	
	if("${CONFIG_NXFONT_SERIF38X48}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_serif38x48.c)
	endif()
	
	# Serif bold fonts
	
	if("${CONFIG_NXFONT_SERIF22X28B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_serif22x28b.c)
	endif()
	
	if("${CONFIG_NXFONT_SERIF27X38B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_serif27x38b.c)
	endif()
	
	if("${CONFIG_NXFONT_SERIF38X49B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_serif38x49b.c)
	endif()
	
	# Pixel fonts
	
	if("${CONFIG_NXFONT_PIXEL_UNICODE}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_pixel-unicode.c)
	endif()
	
	if("${CONFIG_NXFONT_PIXEL_LCD_MACHINE}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_pixel-lcd-machine.c)
	endif()
	
	# X11 misc fixed fonts
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_4X6}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-4x6.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_5X7}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-5x7.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_5X8}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-5x8.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_6X9}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-6x9.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_6X10}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-6x10.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_6X12}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-6x12.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_6X13}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-6x13.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_6X13B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-6x13B.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_6X13O}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-6x13O.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_7X13}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-7x13.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_7X13B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-7x13B.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_7X13O}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-7x13O.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_7X14}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-7x14.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_7X14B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-7x14B.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_8X13}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-8x13.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_8X13B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-8x13B.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_8X13O}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-8x13O.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_9X15}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-9x15.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_9X15B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-9x15B.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_9X18}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-9x18.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_9X18B}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-9x18B.c)
	endif()
	
	if("${CONFIG_NXFONT_X11_MISC_FIXED_10X20}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_bitmaps_x11-misc-fixed-10x20.c)
	endif()
	
	# Tom Thumb mono-space 4x6 font
	
	if("${CONFIG_NXFONT_TOM_THUMB_4X6}" STREQUAL "y")
		list(APPEND CSRCS nxfonts_tom-thumb-4x6.c)
	endif()
	
	# Add the nxfont/ directory to the build

	set_parent_srcs(${CSRCS})

endif()
