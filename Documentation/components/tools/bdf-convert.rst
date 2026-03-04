=================
``bdf-convert.c``
=================

This C file is used to build the bdf-converter program.  The bdf-converter
program can be used to convert fonts in Bitmap Distribution Format (BDF)
into fonts that can be used in the NX graphics system.

Below are general instructions for creating and installing a new font
in the NX graphic system:

1. Locate a font in BDF format,
2. Use the bdf-converter program to convert the BDF font to the NuttX
   font format.  This will result in a C header file containing
   definitions.  That header file should be installed at, for example,
   libnx/nxfonts/nxfonts_myfont.h.

Create a new NuttX configuration variable.  For example, suppose
you define the following variable:  CONFIG_NXFONT_MYFONT.  Then
you would need to:

3. Define CONFIG_NXFONT_MYFONT=y in your NuttX configuration file.

A font ID number has to be assigned for each new font.  The font ID
is defined in the file include/nuttx/nx/nxfonts.h.  Those definitions
have to be extended to support your new font.  Look at how the font ID
enabled by CONFIG_NXFONT_SANS23X27 is defined and add an ID for your
new font in a similar fashion:

4. include/nuttx/nx/nxfonts.h. Add your new font as a possible system
   default font::

         #if defined(CONFIG_NXFONT_SANS23X27)
         # define NXFONT_DEFAULT FONTID_SANS23X27
         #elif defined(CONFIG_NXFONT_MYFONT)
         # define NXFONT_DEFAULT FONTID_MYFONT
         #endif

Then define the actual font ID.  Make sure that the font ID value
is unique::

         enum nx_fontid_e
          {
           FONTID_DEFAULT     = 0      /* The default font */
           #ifdef CONFIG_NXFONT_SANS23X27
           , FONTID_SANS23X27 = 1      /* The 23x27 sans serif font */
           #endif
           #ifdef CONFIG_NXFONT_MYFONT
           , FONTID_MYFONT    = 2      /* My shiny, new font */
           #endif
           ...

Now add the font to the NX build system.  There are several files that
you have to modify to do this.  Look how the build system uses the
font CONFIG_NXFONT_SANS23X27 for examples:

5. nuttx/graphics/Makefile.  This file needs logic to auto-generate
   a C source file from the header file that you generated with the
   the bdf-converter program.  Notice NXFONTS_FONTID=2; this must be
   set to the same font ID value that you defined in the
   include/nuttx/nx/nxfonts.h file::

       genfontsources:
         ifeq ($(CONFIG_NXFONT_SANS23X27),y)
          @$(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=1 EXTRAFLAGS=$(EXTRAFLAGS)
        endif
         ifeq ($(CONFIG_NXFONT_MYFONT),y)
          @$(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=2 EXTRAFLAGS=$(EXTRAFLAGS)
        endif

6. nuttx/libnx/nxfonts/Make.defs.  Set the make variable NXFSET_CSRCS.
   NXFSET_CSRCS determines the name of the font C file to build when
   NXFONTS_FONTID=2::

         ifeq ($(CONFIG_NXFONT_SANS23X27),y)
         NXFSET_CSRCS    += nxfonts_bitmaps_sans23x27.c
         endif
         ifeq ($(CONFIG_NXFONT_MYFONT),y)
         NXFSET_CSRCS    += nxfonts_bitmaps_myfont.c
         endif

7. nuttx/libnx/nxfonts/Makefile.sources.  This is the Makefile used
   in step 5 that will actually generate the font C file.  So, given
   your NXFONTS_FONTID=2, it needs to determine a prefix to use for
   auto-generated variable and function names and (again) the name of
   the auto-generated file to create (this must be the same name that
   was used in nuttx/libnx/nxfonts/Make.defs)::

         ifeq ($(NXFONTS_FONTID),1)
         NXFONTS_PREFIX    := g_sans23x27_
         GEN_CSRC    = nxfonts_bitmaps_sans23x27.c
         endif
         ifeq ($(NXFONTS_FONTID),2)
         NXFONTS_PREFIX    := g_myfont_
         GEN_CSRC    = nxfonts_bitmaps_myfont.c
         endif

8. graphics/libnx/nxfonts_bitmaps.c.  This is the file that contains
   the generic font structures.  It is used as a "template" file by
   nuttx/libnx/nxfonts/Makefile.sources to create your customized
   font data set::

         #if NXFONTS_FONTID == 1
         #  include "nxfonts_sans23x27.h"
         #elif NXFONTS_FONTID == 2
         #  include "nxfonts_myfont.h"
         #else
         #  error "No font ID specified"
         #endif

   Where nxfonts_myfont.h is the NuttX font file that we generated in
   step 2 using the bdf-converter tool.

9. libnx/nxfonts/nxfonts_getfont.c.  Finally, we need to extend the
   logic that does the run-time font lookups so that can find our new
   font.  The lookup function is NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid).
   The new font information needs to be added to data structures used by
   that function::

        #ifdef CONFIG_NXFONT_SANS23X27
         extern const struct nx_fontpackage_s g_sans23x27_package;
         #endif
         #ifdef CONFIG_NXFONT_MYFONT
         extern const struct nx_fontpackage_s g_myfont_package;
         #endif

         static FAR const struct nx_fontpackage_s *g_fontpackages[] =
         {
         #ifdef CONFIG_NXFONT_SANS23X27
         &g_sans23x27_package,
         #endif
         #ifdef CONFIG_NXFONT_MYFONT
         &g_myfont_package,
         #endif
         NULL
         };

