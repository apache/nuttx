README
^^^^^^

This directory contains tiny graphics support for NuttX.  The contents of this directory
are only build if CONFIG_NX is defined in the NuttX configuration file.

Contents
^^^^^^^^
  Roadmap
  Related Header Files
  Directories
  Installing New Fonts
  Configuration Settings

Roadmap
^^^^^^^

This directory holds NuttX graphic packages.  Not all of these packages are implemented
at the present, but here is the longer term roadmap:

  NxWidgets - NxWidgets is a higher level, C++, object-oriented library for object-
              oriented access to graphics "widgets."  NxWidgets is provided as a separate
              package.  NxWidgets is built on top of the core NuttX graphics subsystem,
              but is not a part of the core graphics subsystems.
  NXTOOLKIT - A set of C graphics tools that provide higher-level window drawing
              operations.  The toolkit can be used for window-oriented graphics
              without NxWidgets and is built on top of NX.
  NXFONTS   - A set of C graphics tools for presenting (bitmap) font images.
  NX        - The tiny NuttX windowing system.  This includes the small-footprint
              multi-user implementation (NXMU as described below).  NX can be used
              without NxWidgets and without NXTOOLKIT for raw access to window memory.
  NXGLIB    - Low level graphics utilities and direct framebuffer rendering logic.
              NX is built on top of NXGLIB.
  NxTerm    - NxTerm is a write-only character device that is built on top of
              an NX window.  This character device can be used to provide stdout
              and stderr and, hence, can provide the output side of NuttX console.


Related Header Files
^^^^^^^^^^^^^^^^^^^^

include/nuttx/nx/nxglib.h   -- Describes the NXGLIB C interfaces
include/nuttx/nx/nx.h       -- Describes the NX C interfaces
include/nuttx/nx/nxtk.h     -- Describe the NXTOOLKIT C interfaces
include/nuttx/nx/nxfont.h   -- Describe sthe NXFONT C interfaces

Directories
^^^^^^^^^^^

  The graphics capability consist both of components internal to the RTOS
  and of user-callable interfaces.  In the NuttX kernel mode build there are
  some components of the graphics subsystem are callable in user mode and other
  components that are internal to the RTOS.  This directory, nuttx/graphics,
  contains only those components that are internal to the RTOS.

  User callable functions must, instead, be part of a library that can be
  linked against user applications.  This user callable interfaces are
  provided in sub-directories under nuttx/libs/libnx.

libs/libnx/nx
  Client application callable interfaces.

graphics/nxglib
libs/libnx/nxglib
  The NuttX tiny graphics library.  The directory contains generic utilities
  support operations on primitive graphics objects and logic to rasterize directly
  into a framebuffer.  It has no concept of windows (other than the one, framebuffer
  window).

graphics/nxbe
  This is the "back-end" of a tiny windowing system.  It contains most
  of the important window management logic:  clipping, window controls,
  window drawing, etc.  Currently, the NXserver is the only "front-end"
  (Historically, there was a single user front-end, NXSU, but that front-
  end no longer exists).

graphics/nxmu
libs/libnx/nxmu
  This is the NX multi user "front end".  When combined with the generic
  "back-end" (nxbe), it implements a multi-threaded, multi-user windowing
  system.  The files in this directory present the window APIs described in
  include/nuttx/nx/nx.h.  The multi-user front-end includes the NX graphics
  server that executes on its own thread;  multiple graphics clients then
  communicate with the server via a POSIX message queue to serialize window
  operations from many threads. The multi-user front-end is selected
  automatically.

libs/libnx/nxfonts
  This is where the NXFONTS implementation resides.  This is a relatively low-
  level set of charset set/glyph management APIs.  See include/nuttx/nx/nxfonts.h

libs/libnx/nxtk
  This is where the NXTOOLKIT implementation resides.  This toolkit is built on
  top of NX and works with either the single-user or multi-user NX version. See
  include/nuttx/nx/nxtk.h

apps/grahpics/nxwidgets
  The NxWidgets code is provided as a separate package located outside of the
  NuttX source tree (probably at this location).

graphics/vnc
  The future home of the VNC Remote Frame Buffer (RFB) server and client
  implementations.

Installing New Fonts
^^^^^^^^^^^^^^^^^^^^

  [Refer to nuttx/libs/libnx/nxfonts/README.txt]

Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^

General NX Settings
-------------------

CONFIG_NX
  Enables overall support for graphics library and NX
CONFIG_NX_NPLANES
  Some YUV color formats requires support for multiple planes, one for each
  color component.  Unless you have such special hardware, this value should be
  undefined or set to 1.
CONFIG_NX_WRITEONLY
  Define if the underlying graphics device does not support read operations.
  Automatically defined if CONFIG_NX_LCDDRIVER and CONFIG_LCD_NOGETRUN are
  defined.
CONFIG_NX_DISABLE_1BPP, CONFIG_NX_DISABLE_2BPP,
CONFIG_NX_DISABLE_4BPP, CONFIG_NX_DISABLE_8BPP,
CONFIG_NX_DISABLE_16BPP, CONFIG_NX_DISABLE_24BPP, and
CONFIG_NX_DISABLE_32BPP
  NX supports a variety of pixel depths.  You can save some memory by disabling
  support for unused color depths.
CONFIG_NX_PACKEDMSFIRST
  If a pixel depth of less than 8-bits is used, then NX needs to know if the
  pixels pack from the MS to LS or from LS to MS
CONFIG_NX_XYINPUT
  Build in support for a X/Y positional input device such as a mouse or a
  touchscreen.
CONFIG_NX_KBD
  Build in support of keypad/keyboard input.
CONFIG_NXTK_BORDERWIDTH
  Specifies the width of the border (in pixels) used with framed windows.
  The default is 4.
CONFIG_NXTK_BORDERCOLOR1, CONFIG_NXTK_BORDERCOLOR2, CONFIG_NXTK_BORDERCOLOR3
  Specify the colors of the border used with framed windows.
  CONFIG_NXTK_BORDERCOLOR2 is the shadow side color and so is normally darker.
  CONFIG_NXTK_BORDERCOLOR3 is the shiny side color and so is normally brighter.
  The default is mediumdark grey, and light grey, respectively
CONFIG_NXTK_AUTORAISE
  If set, a window will be raised to the top if the mouse position is over a
  visible portion of the window.  Default: A mouse button must be clicked over
  a visible portion of the window.
CONFIG_VNCSERVER and CONFIG_VNCCLIENT
  Enable the VNC RFB server and client, respecitively.

Font Selections
---------------

  [Refer to nuttx/libs/libnx/nxfonts/README.txt]

NxTerm Configuration Settings
--------------------------------

CONFIG_NXTERM
  Enables building of the NxTerm driver.

NxTerm output text/graphics options:

CONFIG_NXTERM_BPP
  Currently, NxTerm supports only a single pixel depth. This
  configuration setting must be provided to support that single pixel depth.
  Default: The smallest enabled pixel depth. (see CONFIG_NX_DISABLE_*BPP)
CONFIG_NXTERM_CURSORCHAR
  The bitmap code to use as the cursor.  Default '_'
CONFIG_NXTERM_MXCHARS
  NxTerm needs to remember every character written to the console so
  that it can redraw the window. This setting determines the size of some
  internal memory allocations used to hold the character data. Default: 128.
CONFIG_NXTERM_CACHESIZE
  NxTerm supports caching of rendered fonts. This font caching is required
  for two reasons: (1) First, it improves text performance, but more
  importantly (2) it preserves the font memory. Since the NX server runs on
  a separate server thread, it requires that the rendered font memory persist
  until the server has a chance to render the font. Unfortunately, the font
  cache would be quite large if all fonts were saved. The CONFIG_NXTERM_CACHESIZE
  setting will control the size of the font cache (in number of glyphs). Only that
  number of the most recently used glyphs will be retained. Default: 16.
  NOTE: There can still be a race condition between the NxTerm driver and the
  NX task.  If you every see character corruption (especially when printing
  a lot of data or scrolling), then increasing the value of CONFIG_NXTERM_CACHESIZE
  is something that you should try.  Alternatively, you can reduce the size of
  CONFIG_MQ_MAXMSGSIZE which will force NxTerm task to pace the server task.
  CONFIG_NXTERM_CACHESIZE should be larger than CONFIG_MQ_MAXMSGSIZE in any event.
CONFIG_NXTERM_LINESEPARATION
  This the space (in rows) between each row of test.  Default: 0
CONFIG_NXTERM_NOWRAP
  By default, lines will wrap when the test reaches the right hand side
  of the window. This setting can be defining to change this behavior so
  that the text is simply truncated until a new line is  encountered.

NxTerm Input options

CONFIG_NXTERM_NXKBDIN
  Take input from the NX keyboard input callback.  By default, keyboard
  input is taken from stdin (/dev/console).  If this option is set, then
  the interface nxterm_kdbin() is enabled.  That interface may be driven
  by window callback functions so that keyboard input *only* goes to the
  top window.
CONFIG_NXTERM_KBDBUFSIZE
  If CONFIG_NXTERM_NXKBDIN is enabled, then this value may be used to
  define the size of the per-window keyboard input buffer.  Default: 16
CONFIG_NXTERM_NPOLLWAITERS
  The number of threads that can be waiting for read data available.
  Default: 4

NX Multi-user options
---------------------

CONFIG_NX_BLOCKING
  Open the client message queues in blocking mode.  In this case,
  nx_eventhandler() will not return until a message is received and processed.
CONFIG_NX_MXSERVERMSGS and CONFIG_NX_MXCLIENTMSGS
  Specifies the maximum number of messages that can fit in the message queues.
  No additional resources are allocated, but this can be set to prevent
  flooding of the client or server with too many messages (CONFIG_PREALLOC_MQ_MSGS
  controls how many messages are pre-allocated).
