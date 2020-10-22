========
Appendix
========

``graphics/`` Directory Structure
=================================

The graphics capability consist both of components internal to the RTOS
and of user-callable interfaces. In the NuttX kernel mode build there
are some components of the graphics subsystem are callable in user mode
and other components that are internal to the RTOS. The directory
``nuttx/graphics`` contains only those components that are internal to
the RTOS. User callable functions must be part of a library that can be
linked against user applications. This user callable interfaces are
provided in sub-directories under ``nuttx/libnx``.

``libnx/nx``
   Common callable interfaces that are, logically, part of both nxmu and
   nxsu.
``graphics/nxglib`` and ``libnx/nxglib``
   The NuttX tiny graphics library. The directory contains generic
   utilities support operations on primitive graphics objects and logic
   to rasterize directly into a framebuffer or through an LCD driver
   interface. It has no concept of windows (other than the one,
   framebuffer or LCD window).
``graphics/nxbe``
   This is the *back-end* of a tiny windowing system. It can be used
   with either of two front-ends to complete a windowing system (see
   ``nxmu`` and ``nxsu`` below). It contains most of the important
   window management logic: clipping, window controls, window drawing,
   etc.
``graphics/nxmu`` and ``libnx/nxmu``
   This is the NX multi user *front end*. When combined with the generic
   *back-end* (``nxbe``), it implements a multi-threaded, multi-user
   windowing system. The files in this directory present the window APIs
   described in ``include/nuttx/nx/nx.h``. The multi-user front end
   includes a graphics server that executes on its own thread; multiple
   graphics clients then communicate with the server via a POSIX message
   queue to serialize window operations from many threads.
``libnx/nxfonts``
   This is where the NXFONTS implementation resides. This is a
   relatively low-level set of charset set/glyph management APIs. See
   ``include/nuttx/nx/nxfonts.h``.
``libnx/nxtk``
   This is where the NXTOOLKIT implementation resides. This toolkit is
   built on top of NX and works with the multi-user NX front-end. See
   ``include/nuttx/nx/nxtk.h``.
``apps/graphics/NxWidgets``
   The :ref:`NxWidgets <nxwidgets>` code is provided as a separate
   package provided in the ``apps/`` repository.
``graphics/nxterm``
   The NxTerm driver is built on top of NX and works with the multi-user
   NX front-end. See ``include/nuttx/nx/nxterm.h``.

NX Configuration Options
========================

General Configuration Settings
------------------------------

``CONFIG_NX``
   Enables overall support for graphics library and NX
``CONFIG_NX_RAMBACKED``
   Enables RAM backed window support. If this option is selected, then
   windows may be optionally created with a RAM framebuffer backing up
   the window content. Rending into the window will result in rending
   into the backup framebuffer, then updating the physical display from
   the framebuffer.

   The advantage of this option is that the application that manages
   window will no longer receive redraw() callbacks. Those calls
   normally occur, for example, when a window "above" moves exposing a
   portion of the window below. If this option is selected, then the
   system will redraw the exposed portion of the window from the backup
   framebuffer without intervention of the window applications. This
   greatly reduces the complexity of the application and performance of
   the window at the expense of increased memory usage.

   An exception is the case when the window is resized to a wider and/or
   taller size. In that case, the redraw callback will till occur. It is
   necessary in that case to provide new graphic content for the
   extended window area.

   Redraw requests in other cases are also suppressed: Changes to window
   position, size, etc.

NXGL Configuration Settings
---------------------------

``CONFIG_NX_NPLANES``:
   Some YUV color formats requires support for multiple planes, one for
   each color component. Unless you have such special hardware, this
   value should be undefined or set to 1.
``CONFIG_NX_DISABLE_1BPP``, ``CONFIG_NX_DISABLE_2BPP``, ``CONFIG_NX_DISABLE_4BPP``, ``CONFIG_NX_DISABLE_8BPP`` ``CONFIG_NX_DISABLE_16BPP``, ``CONFIG_NX_DISABLE_24BPP``, and ``CONFIG_NX_DISABLE_32BPP``:
   NX supports a variety of pixel depths. You can save some memory by
   disabling support for unused color depths.
``CONFIG_NX_PACKEDMSFIRST``:
   If a pixel depth of less than 8-bits is used, then NX needs to know
   if the pixels pack from the MS to LS or from LS to MS
``CONFIG_NX_LCDDRIVER``:
   By default, NX builds to use a framebuffer driver (see
   ``include/nuttx/video/fb.h``). If this option is defined, NX will
   build to use an LCD driver (see ``include/nuttx/lcd/lcd.h``).
``CONFIG_NX_ANTIALIASING``:
   Enable support for anti-aliasing when rendering lines as various
   orientations. This option is only available for use with frame buffer
   drivers and only with 16-, 24-, or 32-bit RGB color formats.

Configuration Settings
----------------------

``CONFIG_NX_XYINPUT``:
   Build in support for an X/Y input such as a mouse or a touscreen.
``CONFIG_NX_KBD``:
   Build in support of keypad/keyboard input.
``CONFIG_NX_WRITEONLY``:
   Define if the underlying graphics device does not support read
   operations. Automatically defined if ``CONFIG_NX_LCDDRIVER`` and
   ``CONFIG_LCD_NOGETRUN`` are defined.

NX Server Configuration Settings
--------------------------------

``CONFIG_NX_BLOCKING``
   Open the client message queues in blocking mode. In this case,
   ``nx_eventhandler()`` will not return until a message is received and
   processed.
``CONFIG_NX_MXSERVERMSGS`` and ``CONFIG_NX_MXCLIENTMSGS``
   Specifies the maximum number of messages that can fit in the message
   queues. No additional resources are allocated, but this can be set to
   prevent flooding of the client or server with too many messages
   (``CONFIG_PREALLOC_MQ_MSGS`` controls how many messages are
   pre-allocated).

NXTK Configuration Settings
---------------------------

``CONFIG_NXTK_BORDERWIDTH``:
   Specifies the width of the border (in pixels) used with framed
   windows. The default is 4.
``CONFIG_NXTK_BORDERCOLOR1``, ``CONFIG_NXTK_BORDERCOLOR2``, and ``CONFIG_NXTK_BORDERCOLOR3``:
   Specify the colors of the border used with framed windows.
``CONFIG_NXTK_BORDERCOLOR2``
   The shadow side color and so is normally darker.
``CONFIG_NXTK_BORDERCOLOR3``
   The shiny side color and so is normally brighter. The default is
   medium, dark, and light grey, respectively
``CONFIG_NXTK_AUTORAISE``:
   If set, a window will be raised to the top if the mouse position is
   over a visible portion of the window. Default: A mouse button must be
   clicked over a visible portion of the window.

NXFONTS Configuration Settings
------------------------------

``CONFIG_NXFONTS_CHARBITS``:
   The number of bits in the character set. Current options are only 7
   and 8. The default is 7.
``CONFIG_NXFONT_SANS17X22``:
   This option enables support for a tiny, 17x22 san serif font (font
   ``ID FONTID_SANS17X22`` == 14).
``CONFIG_NXFONT_SANS20X26``:
   This option enables support for a tiny, 20x26 san serif font (font
   ``ID FONTID_SANS20X26`` == 15).
``CONFIG_NXFONT_SANS23X27``:
   This option enables support for a tiny, 23x27 san serif font (font
   ``ID FONTID_SANS23X27`` == 1).
``CONFIG_NXFONT_SANS22X29``:
   This option enables support for a small, 22x29 san serif font (font
   ``ID FONTID_SANS22X29`` == 2).
``CONFIG_NXFONT_SANS28X37``:
   This option enables support for a medium, 28x37 san serif font (font
   ``ID FONTID_SANS28X37`` == 3).
``CONFIG_NXFONT_SANS39X48``:
   This option enables support for a large, 39x48 san serif font (font
   ``ID FONTID_SANS39X48`` == 4).
``CONFIG_NXFONT_SANS17X23B``:
   This option enables support for a tiny, 17x23 san serif bold font
   (font ``ID FONTID_SANS17X23B`` == 16).
``CONFIG_NXFONT_SANS20X27B``:
   This option enables support for a tiny, 20x27 san serif bold font
   (font ``ID FONTID_SANS20X27B`` == 17).
``CONFIG_NXFONT_SANS22X29B``:
   This option enables support for a small, 22x29 san serif bold font
   (font ID ``FONTID_SANS22X29B`` == 5).
``CONFIG_NXFONT_SANS28X37B``:
   This option enables support for a medium, 28x37 san serif bold font
   (font ID ``FONTID_SANS28X37B`` == 6).
``CONFIG_NXFONT_SANS40X49B``:
   This option enables support for a large, 40x49 san serif bold font
   (font ID ``FONTID_SANS40X49B`` == 7).
``CONFIG_NXFONT_SERIF22X29``:
   This option enables support for a small, 22x29 font (with serifs)
   (font ID ``FONTID_SERIF22X29`` == 8).
``CONFIG_NXFONT_SERIF29X37``:
   This option enables support for a medium, 29x37 font (with serifs)
   (font ID ``FONTID_SERIF29X37`` == 9).
``CONFIG_NXFONT_SERIF38X48``:
   This option enables support for a large, 38x48 font (with serifs)
   (font ID ``FONTID_SERIF38X48`` == 10).
``CONFIG_NXFONT_SERIF22X28B``:
   This option enables support for a small, 27x38 bold font (with
   serifs) (font ID ``FONTID_SERIF22X28B`` == 11).
``CONFIG_NXFONT_SERIF27X38B``:
   This option enables support for a medium, 27x38 bold font (with
   serifs) (font ID ``FONTID_SERIF27X38B`` == 12).
``CONFIG_NXFONT_SERIF38X49B``:
   This option enables support for a large, 38x49 bold font (with
   serifs) (font ID ``FONTID_SERIF38X49B`` == 13).

NxTerm Configuration Settings
-----------------------------

General NxTerm settings.

``CONFIG_NXTERM``:
   Enables building of the NxTerm driver.

NxTerm output text/graphics options:

``CONFIG_NXTERM_BPP``:
   Currently, NxTerm supports only a single pixel depth. This
   configuration setting must be provided to support that single pixel
   depth. Default: The smallest enabled pixel depth. (see
   ``CONFIG_NX_DISABLE_*BPP``)
``CONFIG_NXTERM_CURSORCHAR``:
   The bitmap code to use as the cursor. Default '_'
``CONFIG_NXTERM_MXCHARS``:
   NxTerm needs to remember every character written to the console so
   that it can redraw the window. This setting determines the size of
   some internal memory allocations used to hold the character data.
   Default: 128.
``CONFIG_NXTERM_CACHESIZE``:
   NxTerm supports caching of rendered fonts. This font caching is
   required for two reasons: (1) First, it improves text performance,
   but more importantly (2) it preserves the font memory. Since the NX
   server runs on a separate server thread, it requires that the
   rendered font memory persist until the server has a chance to render
   the font. Unfortunately, the font cache would be quite large if all
   fonts were saved. The ``CONFIG_NXTERM_CACHESIZE`` setting will
   control the size of the font cache (in number of glyphs). Only that
   number of the most recently used glyphs will be retained. Default:
   16.

      NOTE: There can still be a race condition between the NxTerm
      driver and the NX task. If you every see character corruption
      (especially when printing a lot of data or scrolling), then
      increasing the value of ``CONFIG_NXTERM_CACHESIZE`` is something
      that you should try. Alternatively, you can reduce the size of
      ``CONFIG_MQ_MAXMSGSIZE`` which will force NxTerm task to pace the
      server task. ``CONFIG_NXTERM_CACHESIZE`` should be larger than
      ``CONFIG_MQ_MAXMSGSIZE`` in any event.

``CONFIG_NXTERM_LINESEPARATION``:
   This the space (in rows) between each row of test. Default: 0
``CONFIG_NXTERM_NOWRAP``:
   By default, lines will wrap when the test reaches the right hand side
   of the window. This setting can be defining to change this behavior
   so that the text is simply truncated until a new line is encountered.

NxTerm input options:

``CONFIG_NXTERM_NXKBDIN``:
   Take input from the NX keyboard input callback. By default, keyboard
   input is taken from stdin (``/dev/console``). If this option is set,
   then the interface\ ``nxterm_kdbin()`` is enabled. That interface may
   be driven by window callback functions so that keyboard input *only*
   goes to the top window.
``CONFIG_NXTERM_KBDBUFSIZE``:
   If ``CONFIG_NXTERM_NXKBDIN`` is enabled, then this value may be used
   to define the size of the per-window keyboard input buffer. Default:
   16
``CONFIG_NXTERM_NPOLLWAITERS``:
   The number of threads that can be waiting for read data available.
   Default: 4

Installing New Fonts
====================

**The BDF Font Converter**. There is a tool called *bdf-converter* in
the directory ``tools/.``. The *bdf-converter* program be used to
convert fonts in Bitmap Distribution Format (BDF) into fonts that can be
used in the NX graphics system. The BDF format most well known as a font
format traditionally used for X-11 bitmap fonts.

   A Note about Font Copyrights: My understanding is that the underlying
   bitmap font data for traditional fonts cannot be copyrighted (the
   same is not true for scalable fonts). This is because a copyright
   covers only the form of delivery of the font and not the underlying
   font content and, at least for the traditional typefaces, the
   underlying font designs are ancient. There could be issues, however,
   if you convert from modern, trademarked images. However, remember
   that I am a programmer not an attorney and that my knowledge of font
   copyright issues is limited to what I glean by Googling.

**Font Installation Steps**, Below are general instructions for creating
and installing a new font in the NX graphic system. The first two steps
only apply if you are using the BDF font converter program.

#. Locate a font in BDF format. There are many good BDF bitmap fonts
   bundled with X-11. See `this
   link <http://www.cl.cam.ac.uk/~mgk25/ucs-fonts.html>`__, as an
   example,

#. Use the *bdf-converter* program to convert the BDF font to the NuttX
   font format. This will result in a C header file containing
   definitions. That header file should be installed at, for example,
   ``graphics/nxfonts/nxfonts_myfont.h``.

The remaining steps apply however you managed to create the NuttX C font
header file. After you have your C font header file, the next thing to
do is to create a new NuttX configuration variable to select the font.
For example, suppose you define the following variable:
``CONFIG_NXFONT_MYFONT``. Then you would need to:

3. Define ``CONFIG_NXFONT_MYFONT=y`` in your NuttX configuration file.

A font ID number has to be assigned for each new font. The font IDs are
defined in the file ``include/nuttx/nx/nxfonts.h``. Those definitions
have to be extended to support your new font. Look at how the font ID
enabled by ``CONFIG_NXFONT_SANS23X27`` is defined and add an ID for
yournew font in a similar fashion:

4. ``include/nuttx/nx/nxfonts.h``. Add you new font as a possible
   system default font:

   .. code-block:: c

    #if defined(CONFIG_NXFONT_SANS23X27)
    # define NXFONT_DEFAULT FONTID_SANS23X27
    #elif defined(CONFIG_NXFONT_MYFONT)
    # define NXFONT_DEFAULT FONTID_MYFONT
    #endif

   Then define the actual font ID. Make sure that the font ID value is
   unique:

   .. code-block:: c

    #if defined(CONFIG_NXFONT_SANS23X27)
    # define NXFONT_DEFAULT FONTID_SANS23X27
    #elif defined(CONFIG_NXFONT_MYFONT)
    # define NXFONT_DEFAULT FONTID_MYFONT
    #endif

New Add the font to the NX build system. There are several files that
you have to modify to do this. Look how the build system uses the font
CONFIG_NXFONT_SANS23X27 for examaples:

5. ``nuttx/graphics/Makefile``. This file needs logic to
   auto-generate a C source file from the header file that you generated
   with the *bdf-converter* program. Notice ``NXFONTS_FONTID=2``; this
   must be set to the same font ID value that you defined in the
   ``include/nuttx/nx/nxfonts.h`` file.

   .. code-block:: makefile

    genfontsources:
      ifeq ($(CONFIG_NXFONT_SANS23X27),y)
          @$(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=1 EXTRAFLAGS=$(EXTRAFLAGS)
      endif
      ifeq ($(CONFIG_NXFONT_MYFONT),y)
          @$(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=2 EXTRAFLAGS=$(EXTRAFLAGS)
      endif

6. ``nuttx/graphics/nxfonts/Make.defs``. Set the make variable
   ``NXFSET_CSRCS``. ``NXFSET_CSRCS`` determines the name of the font C
   file to build when ``NXFONTS_FONTID=2``:

   .. code-block:: makefile

    ifeq ($(CONFIG_NXFONT_SANS23X27),y)
    NXFSET_CSRCS += nxfonts_bitmaps_sans23x27.c
    endif
    ifeq ($(CONFIG_NXFONT_MYFONT),y)
    NXFSET_CSRCS += nxfonts_bitmaps_myfont.c
    endif

7. ``nuttx/graphics/nxfonts/Makefile.sources``. This is the Makefile
   used in step 5 that will actually generate the font C file. So, given
   your NXFONTS_FONTID=2, it needs to determine a prefix to use for
   auto-generated variable and function names and (again) the name of
   the autogenerated file to create (this must be the same name that was
   used in ``nuttx/graphics/nxfonts/Make.defs``):

   .. code-block:: makefile

    ifeq ($(NXFONTS_FONTID),1)
    NXFONTS_PREFIX  := g_sans23x27_
    GEN_CSRC  = nxfonts_bitmaps_sans23x27.c
    endif
    ifeq ($(NXFONTS_FONTID),2)
    NXFONTS_PREFIX  := g_myfont_
    GEN_CSRC  = nxfonts_bitmaps_myfont.c
    endif

8. ``graphics/nxfonts/nxfonts_bitmaps.c``. This is the file that
   contains the generic font structures. It is used as a "template&qout;
   file by ``nuttx/graphics/nxfonts/Makefile.sources``\ to create your
   customized font data set at build time.

   .. code-block:: c

    #if NXFONTS_FONTID == 1
    #  include "nxfonts_sans23x27.h"
    #elif NXFONTS_FONTID == 2
    #  include "nxfonts_myfont.h"
    #else
    #  error "No font ID specified"
    #endif

   Where ``nxfonts_myfont.h`` is the NuttX font file that we generated
   in step 2 using the *bdf-converter* tool.

9. ``graphics/nxfonts/nxfonts_getfont.c``. Finally, we need to
   extend the logic that does the run-time font lookups so that can find
   our new font. The lookup function is
   ```NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid)`` <#nxfgetfonthandle>`__.
   Note that the lookup is based on the font ID that was defined in step
   4. The new font information needs to be added to data structures used
   by that function:

   .. code-block:: c

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


NX Test Coverage
================

``apps/examples/nx``. The primary test tool for debugging NX resides
at ``apps/examples/nx``.

**Building** ``apps/examples/nx``. NX testing was performed using
``apps/examples/nx`` with the Linux/Cygwin-based NuttX simulator.
Configuration files for building this test can be found in
``boards/sim/sim/sim/configs/nx`` and
``boards/sim/sim/sim/configs/nx11``. There are two alternative
configurations for building the simulation:

#. The configuration using the configuration file at
   ``boards/sim/sim/sim/configs/nx/defconfig``. This default
   configuration exercises the NX logic a 8 BPP but provides no visual
   feedback. In this configuration, a very simple, simulated framebuffer
   driver is used that is based upon a simple region of memory posing as
   video memory. That default configuration can be built as follows::

    tools/configure.sh sim:nx
    make
    ./nuttx

#. The preferred configuration is at
   ``boards/sim/sim/sim/configs/nx11/defconfig``. This configuration
   extends the test with a simulated framebuffer driver that uses an X
   window as a framebuffer. This is a superior test configuration
   because the X window appears at your desktop and you can see the NX
   output. This preferred configuration can be built as follows::

    tools/configure sim:nx11
    make
    ./nuttx

   *Update:* The sim target has suffered some bit-rot over the years and
   so the following caveats need to be added:

   -  The X target builds under recent Cygwin configurations, but does
      not execute. (It fails inside of ``XOpenDisplay()``.

   -  The X target does not build under current (9.09) Ubuntu
      distributions. I needed to make the following changes:

      The build will also fail to locate the X header files unless you
      install an X11 development package.

   -  Refer to the readme file in sim configuration
      `README.txt <https://github.com/apache/incubator-nuttx/blob/master/boards/sim/sim/sim/README.txt>`__
      file for additional information.

**Test Coverage**. At present, ``apps/examples/nx``\ t only exercises a
subset of NX; the remainder is essentially untested. The following table
describes the testing performed on each NX API:

NXGLIB API Test Coverage
------------------------

================================ ==================================== ========
Function                         Special Setup/Notes                  Verified
================================ ==================================== ========
``nxgl_rgb2yuv()``               .                                    NO
``nxgl_yuv2rgb()``               .                                    NO
``nxgl_rectcopy()``              .                                    YES
``nxgl_rectoffset()``            .                                    YES
``nxgl_vectoradd()``             .                                    YES
``nxgl_vectorsubtract()``        .                                    YES
``nxgl_rectintersect()``         .                                    YES
``nxgl_rectunion()``             .                                    YES
``nxgl_nonintersecting()``       .                                    YES
``nxgl_rectoverlap()``           .                                    YES
``nxgl_rectinside()``            .                                    YES
``nxgl_rectsize()``              .                                    YES
``nxgl_nullrect()``              .                                    YES
``nxgl_runoffset()``             Verified by apps/examples/nxlines.   YES
``nxgl_runcopy()``               .                                    NO
``nxgl_trapoffset()``            Verified by apps/examples/nxlines.   YES
``nxgl_trapcopy()``              Verified by apps/examples/nxlines.   YES
``nxgl_colorcopy``               .                                    YES
``nxgl_splitline``               Verified using apps/examples/nxlines YES
                                 Generally works well, but has some
                                 accuracy/overflow problems wide
                                 lines that are nearly horizontal.
                                 There is a "fudge factor" that seems
                                 to eliminate the problem, but there
                                 could still be issues in some
                                 configurations.
``nxgl_circlepts``               Verified by apps/examples/nxlines.   YES
``nxgl_circletraps``             Verified by apps/examples/nxlines.   YES
================================ ==================================== ========

NX Server Callbacks Test Coverage
---------------------------------

============== ==================== ========
Function       Special Setup/Notes  Verified
============== ==================== ========
``redraw()``   .                    YES
``position()`` .                    YES
``mousein()``  .                    YES
``kbdin()``    .                    YES
============== ==================== ========

NX API Test Coverage
--------------------

========================= ===============================================================  ========
Function                  Special Setup/Notes                                              Verified
========================= ===============================================================  ========
``nx_runinstance()``      .                                                                YES
``nx_connectinstance()``  .                                                                YES
``nx_disconnect()``       .                                                                YES
``nx_eventhandler()``     .                                                                YES
``nx_eventnotify()``      This is not used in the current version of apps/examples/nx,     NO
                          was tested in a previous version)
``nx_openwindow()``       Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y`` in the             YES
                          ``<NuttX-Directory>/.config file``
``nx_closewindow()``      Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y``                    YES
                          in the ``<NuttX-Directory>/.config`` file
``nx_requestbkgd()``      Verified by ``apps/examples/nxtext`` and                         YES
                          ``apps/examples/nxhello``.
``nx_releasebkgd()``      Verified by ``apps/examples/nxtext`` and                         YES
                          ``apps/examples/nxhello``.
``nx_getposition()``      .                                                                NO
``nx_setposition()``      Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y`` in the             YES
                          ``<NuttX-Directory>/.config`` file
``nx_setsize()``          Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y`` in the             YES
                          ``<NuttX-Directory>/.config`` file
``nx_raise()``            Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y`` in the             YES
                          ``<NuttX-Directory>/.config`` file
``nx_lower()``            Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y`` in the             YES
                          ``<NuttX-Directory>/.config`` file
``nx_modal()``            .                                                                NO
``nx_setvisibility()``    Exercized using Twm4Nx                                           YES, Informally
``nx_ishidden()``         Exercized using Twm4Nx                                           YES, Informally
``nx_fill()``             Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y`` in the             YES
                          ``<NuttX-Directory>/.config`` file
``nx_getrectangle()``     .                                                                YES
``nx_filltrapezoid()``    Verified by ``apps/examples/nxlines``.                           YES
``nx_drawline()``         by ``apps/examples/nxlines``.                                    YES
``nx_drawcircle()``       Verified by ``apps/examples/nxlines``.                           YES
``nx_fillcircle()``       Verified by ``apps/examples/nxlines``.                           YES
``nx_setbgcolor()``       .                                                                YES
``nx_move()``             Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y`` in the             YES
                          ``<NuttX-Directory>/.config`` file
``nx_bitmap()``           Change to ``CONFIG_EXAMPLES_NX_RAWWINDOWS=y`` in the             YES
                          ``<NuttX-Directory>/.config`` file.
``nx_kbdin()``            .                                                                YES
``nx_mousein()``          .                                                                YES
========================= ===============================================================  ========

NXTK API Test Coverage
----------------------

============================ ========================= ========
Function                     Special Setup/Notes       Verified
============================ ========================= ========
``nxtk_openwindow()``        .                         YES
``nxtk_closewindow()``       .                         YES
``nxtk_getposition()``       .                         NO
``nxtk_setposition()``       .                         YES
``nxtk_setsize()``           .                         YES
``nxtk_raise()``             .                         YES
``nxtk_lower()``             .                         YES
``nxtk_modal()``             .                         NO
``nxtk_setvisibility()``     Exercized using Twm4Nx    YES, informally
``nxtk_ishidden()``          Exercized using Twm4Nx    YES, informally
``nxtk_fillwindow()``        .                         YES
``nxtk_getwindow()``         .                         NO
``nxtk_filltrapwindow()``    .                         NO
``nxtk_drawlinewindow()``    .                         YES
``nxtk_drawcirclewindow()``  .                         YES
``nxtk_fillcirclewindow()``  .                         YES
``nxtk_movewindow()``        .                         NO
``nxtk_bitmapwindow()``      .                         YES
``nxtk_opentoolbar()``       .                         YES
``nxtk_closetoolbar()``      .                         YES
``nxtk_filltoolbar()``       .                         YES
``nxtk_gettoolbar()``        .                         NO
``nxtk_filltraptoolbar()``   .                         NO
``nxtk_drawlinetoolbar()``   .                         NO
``nxtk_drawcircletoolbar()`` .                         NO
``nxtk_fillcircletoolbar()`` .                         NO
``nxtk_movetoolbar()``       .                         NO
``nxtk_bitmaptoolbar()``     .                         NO
============================ ========================= ========

NXFONTS API Test Coverage
-------------------------

======================== ============================= ========
Function                 Special Setup/Notes           Verified
======================== ============================= ========
``nxf_getfonthandle()``  .                             YES
``nxf_getfontset()``     .                             YES
``nxf_getbitmap()``      .                             YES
``nxf_convert_2bpp()``   .                             NO
``nxf_convert_4bpp()``   .                             NO
``nxf_convert_8bpp()``   Use defconfig when building.  YES
``nxf_convert_16bpp()``  .                             YES
``nxf_convert_24bpp()``  .                             NO
``nxf_convert_32bpp()``  .                             YES
======================== ============================= ========

