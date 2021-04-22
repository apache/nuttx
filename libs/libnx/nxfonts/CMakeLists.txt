#if (CONFIG_NXFONTS)
#  set(SRCS
#    nxfonts_getfont.c nxfonts_cache.c)
#    nxfonts_convert_1bpp.c nxfonts_convert_2bpp.c)
#    nxfonts_convert_4bpp.c nxfonts_convert_8bpp.c)
#    nxfonts_convert_16bpp.c nxfonts_convert_24bpp.c)
#    nxfonts_convert_32bpp.c)

#  # Monospace fonts

#  if (CONFIG_NXFONT_MONO5X8)
#    list(APPEND SRCS nxfonts_bitmaps_mono5x8.c)
#  endif()

#  # Sans serif fonts

#  if (CONFIG_NXFONT_SANS17X22)
#    list(APPEND SRCS nxfonts_bitmaps_sans17x22.c)
#  endif()

#  if (CONFIG_NXFONT_SANS20X26)
#    list(APPEND SRCS nxfonts_bitmaps_sans20x26.c)
#  endif()

#  if (CONFIG_NXFONT_SANS23X27)
#    list(APPEND SRCS nxfonts_bitmaps_sans23x27.c)
#  endif()

#  if (CONFIG_NXFONT_SANS22X29)
#    list(APPEND SRCS nxfonts_bitmaps_sans22x29.c)
#  endif()

#  if (CONFIG_NXFONT_SANS28X37)
#    list(APPEND SRCS nxfonts_bitmaps_sans28x37.c)
#  endif()

#  if (CONFIG_NXFONT_SANS39X48)
#    list(APPEND SRCS nxfonts_bitmaps_sans39x48.c)
#  endif()

#  # Sans serif bold fonts

#  if (CONFIG_NXFONT_SANS17X23B)
#    list(APPEND SRCS nxfonts_bitmaps_sans17x23b.c)
#  endif()

#  if (CONFIG_NXFONT_SANS20X27B)
#    list(APPEND SRCS nxfonts_bitmaps_sans20x27b.c)
#  endif()

#  if (CONFIG_NXFONT_SANS22X29B)
#    list(APPEND SRCS nxfonts_bitmaps_sans22x29b.c)
#  endif()

#  if (CONFIG_NXFONT_SANS28X37B)
#    list(APPEND SRCS nxfonts_bitmaps_sans28x37b.c)
#  endif()

#  if (CONFIG_NXFONT_SANS40X49B)
#    list(APPEND SRCS nxfonts_bitmaps_sans40x49b.c)
#  endif()

#  # Serif fonts

#  if (CONFIG_NXFONT_SERIF22X29)
#    list(APPEND SRCS nxfonts_bitmaps_serif22x29.c)
#  endif()

#  if (CONFIG_NXFONT_SERIF29X37)
#    list(APPEND SRCS nxfonts_bitmaps_serif29x37.c)
#  endif()

#  if (CONFIG_NXFONT_SERIF38X48)
#    list(APPEND SRCS nxfonts_bitmaps_serif38x48.c)
#  endif()

#  # Serif bold fonts

#  if (CONFIG_NXFONT_SERIF22X28B)
#    list(APPEND SRCS nxfonts_bitmaps_serif22x28b.c)
#  endif()

#  if (CONFIG_NXFONT_SERIF27X38B)
#    list(APPEND SRCS nxfonts_bitmaps_serif27x38b.c)
#  endif()

#  if (CONFIG_NXFONT_SERIF38X49B)
#    list(APPEND SRCS nxfonts_bitmaps_serif38x49b.c)
#  endif()

#  # Pixel fonts

#  if (CONFIG_NXFONT_PIXEL_UNICODE)
#    list(APPEND SRCS nxfonts_bitmaps_pixel-unicode.c)
#  endif()

#  if (CONFIG_NXFONT_PIXEL_LCD_MACHINE)
#    list(APPEND SRCS nxfonts_bitmaps_pixel-lcd-machine.c)
#  endif()

#  # X11 misc fixed fonts

#  if (CONFIG_NXFONT_X11_MISC_FIXED_4X6)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-4x6.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_5X7)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-5x7.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_5X8)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-5x8.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_6X9)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-6x9.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_6X10)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-6x10.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_6X12)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-6x12.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_6X13)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-6x13.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_6X13B)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-6x13b.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_6X13O)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-6x13o.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_7X13)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-7x13.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_7X13B)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-7x13b.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_7X13O)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-7x13o.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_7X14)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-7x14.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_7X14B)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-7x14b.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_8X13)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-8x13.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_8X13B)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-8x13b.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_8X13O)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-8x13o.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_9X15)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-9x15.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_9X15B)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-9x15b.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_9X18)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-9x18.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_9X18B)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-9x18b.c)
#  endif()

#  if (CONFIG_NXFONT_X11_MISC_FIXED_10X20)
#    list(APPEND SRCS nxfonts_bitmaps_x11-misc-fixed-10x20.c)
#  endif()

#  # Tom Thumb mono-space 4x6 font

#  if (CONFIG_NXFONT_TOM_THUMB_4X6)
#    list(APPEND SRCS nxfonts_bitmaps_tom-thumb-4x6.c)
#  endif()

#  # Font generation ##########################################################

#  if (NXFONTS_BITSPERPIXEL)
#    if (NXFONTS_BITSPERPIXEL EQUAL 1)
#      set(NXFONTS_SUFFIX _1bpp)
#      set(GENSRC nxfonts_convert_1bpp.c)
#    endif()
#    if (NXFONTS_BITSPERPIXEL EQUAL 2)
#      set(NXFONTS_SUFFIX _2bpp)
#      set(GENSRC nxfonts_convert_2bpp.c)
#    endif()
#    if (NXFONTS_BITSPERPIXEL EQUAL 4)
#      set(NXFONTS_SUFFIX _4bpp)
#      set(GENSRC nxfonts_convert_4bpp.c)
#    endif()
#    if (NXFONTS_BITSPERPIXEL EQUAL 8)
#      set(NXFONTS_SUFFIX _8bpp)
#      set(GENSRC nxfonts_convert_8bpp.c)
#    endif()
#    if (NXFONTS_BITSPERPIXEL EQUAL 16)
#      set(NXFONTS_SUFFIX _16bpp)
#      set(GENSRC nxfonts_convert_16bpp.c)
#    endif()
#    if (NXFONTS_BITSPERPIXEL EQUAL 24)
#      set(NXFONTS_SUFFIX _24bpp)
#      set(GENSRC nxfonts_convert_24bpp.c)
#    endif()
#    if (NXFONTS_BITSPERPIXEL EQUAL 32)
#      set(NXFONTS_SUFFIX _32bpp)
#      set(GENSRC nxfonts_convert_32bpp.c)
#    endif()

#    target_sources(nx PRIVATE nxfonts_convert.c)
#    target_compile_definitions(nx PRIVATE
#      -DNXFONTS_BITSPERPIXEL=${NXFONTS_BITSPERPIXEL}
#      -DNXFONTS_SUFFIX=${NXFONTS_SUFFIX})
#  endif()

#if (NXFONTS_FONTID)
#  # MONO

#  if (NXFONTS_FONTID EQUAL 18)
#    set(NXFONTS_PREFIX g_mono5x8_)
#    set(GENSRC nxfonts_bitmaps_mono5x8.c)
#  endif()

#  # SANS

#  if (NXFONTS_FONTID EQUAL 14)
#    set(NXFONTS_PREFIX g_sans17x22_)
#    set(GENSRC nxfonts_bitmaps_sans17x22.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 15)
#    set(NXFONTS_PREFIX g_sans20x26_)
#    set(GENSRC nxfonts_bitmaps_sans20x26.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 1)
#    set(NXFONTS_PREFIX g_sans23x27_)
#    set(GENSRC nxfonts_bitmaps_sans23x27.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 2)
#    set(NXFONTS_PREFIX g_sans22x29_)
#    set(GENSRC nxfonts_bitmaps_sans22x29.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 3)
#    set(NXFONTS_PREFIX g_sans28x37_)
#    set(GENSRC nxfonts_bitmaps_sans28x37.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 4)
#    set(NXFONTS_PREFIX g_sans39x48_)
#    set(GENSRC nxfonts_bitmaps_sans39x48.c)
#  endif()

#  # SANS-BOLD

#  if (NXFONTS_FONTID EQUAL 16)
#    set(NXFONTS_PREFIX g_sans17x23b_)
#    set(GENSRC nxfonts_bitmaps_sans17x23b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 17)
#    set(NXFONTS_PREFIX g_sans20x27b_)
#    set(GENSRC nxfonts_bitmaps_sans20x27b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 5)
#    set(NXFONTS_PREFIX g_sans22x29b_)
#    set(GENSRC nxfonts_bitmaps_sans22x29b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 6)
#    set(NXFONTS_PREFIX g_sans28x37b_)
#    set(GENSRC nxfonts_bitmaps_sans28x37b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 7)
#    set(NXFONTS_PREFIX g_sans40x49b_)
#    set(GENSRC nxfonts_bitmaps_sans40x49b.c)
#  endif()

#  # SERIF

#  if (NXFONTS_FONTID EQUAL 8)
#    set(NXFONTS_PREFIX g_serif22x29_)
#    set(GENSRC nxfonts_bitmaps_serif22x29.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 9)
#    set(NXFONTS_PREFIX g_serif29x37_)
#    set(GENSRC nxfonts_bitmaps_serif29x37.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 10)
#    set(NXFONTS_PREFIX g_serif38x48_)
#    set(GENSRC nxfonts_bitmaps_serif38x48.c)
#  endif()

#  # SERIF-BOLD

#  if (NXFONTS_FONTID EQUAL 11)
#    set(NXFONTS_PREFIX g_serif22x28b_)
#    set(GENSRC nxfonts_bitmaps_serif22x28b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 12)
#    set(NXFONTS_PREFIX g_serif27x38b_)
#    set(GENSRC nxfonts_bitmaps_serif27x38b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 13)
#    set(NXFONTS_PREFIX g_serif38x49b_)
#    set(GENSRC nxfonts_bitmaps_serif38x49b.c)
#  endif()

#  # Pixel fonts

#  if (NXFONTS_FONTID EQUAL 19)
#    set(NXFONTS_PREFIX g_pixel_unicode_)
#    set(GENSRC nxfonts_bitmaps_pixel-unicode.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 20)
#    set(NXFONTS_PREFIX g_pixel_lcd_machine_)
#    set(GENSRC nxfonts_bitmaps_pixel-lcd-machine.c)
#  endif()

#  # X11-misc-fixed fonts

#  if (NXFONTS_FONTID EQUAL 21)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_4x6_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-4x6.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 22)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_5x7_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-5x7.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 23)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_5x8_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-5x8.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 24)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_6x9_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-6x9.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 25)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_6x10_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-6x10.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 26)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_6x12_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-6x12.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 27)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_6x13_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-6x13.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 28)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_6x13B_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-6x13b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 29)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_6x13O_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-6x13o.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 30)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_7x13_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-7x13.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 31)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_7x13B_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-7x13b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 32)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_7x13O_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-7x13o.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 33)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_7x14_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-7x14.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 34)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_7x14B_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-7x14b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 35)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_8x13_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-8x13.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 36)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_8x13B_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-8x13b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 37)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_8x13O_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-8x13o.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 38)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_9x15_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-9x15.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 39)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_9x15B_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-9x15b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 40)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_9x18_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-9x18.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 41)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_9x18B_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-9x18b.c)
#  endif()

#  if (NXFONTS_FONTID EQUAL 42)
#    set(NXFONTS_PREFIX g_x11_misc_fixed_10x20_)
#    set(GENSRC nxfonts_bitmaps_x11-misc-fixed-10x20.c)
#  endif()

#  # Tom Thumb mono-space 4x6 font

#  if (NXFONTS_FONTID EQUAL 43)
#    set(NXFONTS_PREFIX g_tom_thumb_4x6_)
#    set(GENSRC nxfonts_bitmaps_tom-thumb-4x6.c)
#  endif()

#  DEPENDENCY := nxfonts_bitmaps.c
#  CPPFLAGS += -DNXFONTS_FONTID=$(NXFONTS_FONTID)
#  CPPFLAGS += -DNXFONTS_PREFIX=$(NXFONTS_PREFIX)
#  endif()

#GEN_TMP = $(GEN_CSRC:.c=.i)

#all: $(GEN_CSRC)
#.PHONY : clean distclean

#$(GEN_CSRC) : $(DEPENDENCY)
#	$(call PREPROCESS, $<, $(GEN_TMP))
#	$(Q) cat $(GEN_TMP) | sed -e "/^#/d" >$@
#	$(Q) rm -f  $(GEN_TMP)

#clean:
#	$(call DELFILE, *.i)
#	$(call CLEAN)

#distclean: clean
#	$(call DELFILE, nxfonts_convert_*bpp.c)
#	$(call DELFILE, nxfonts_bitmaps_*.c)

#  target_sources(nx PRIVATE ${SRCS})
#endif()
