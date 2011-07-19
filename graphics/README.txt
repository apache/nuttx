README
^^^^^^

This directory contains tiny graphics support for NuttX.  The contents of this directory
are only build if CONFIG_NX is defined in the NuttX configuration file.

Roadmap
^^^^^^^

This directory holds NuttX graphic packages.  Not all of these packages are implemented
at the present, but here is the longer term roadmap:

  NXWIDGETS - I had originally planned a high level, C++, object-oriented library for
              object-oriented access to graphics widgets.  However, because C++ compilers
              are not available for some of the targets supported by NuttX, I have
              decided to implement the entire solution in  C -- that makes the solution
              much uglier, but works fine on all platforms.
  NXTOOLKIT - A set of C graphics tools that provide higher-level window drawing
              operations.  The toolkit can be used for window-oriented graphics
              without NXWIDGETS and is built on top of NX.
  NXFONTS   - A set of C graphics tools for present (bitmap) font images.
  NX        - The tiny NuttX windowing system.  This includes both a small-footprint,
              single user implementaton (NXSU as described below) and a somewhat
              larger multi-user implentation (NXMU as described below).  Both
              conform to the same APIs as defined in include/nuttx/nx.h and, hence,
              are more-or-less interchangable.  NX can be used without NXWIDGETS
              and without NXTOOLKIT for raw access to window memory.
  NXGLIB    - Low level graphics utilities and direct framebuffer rendering logic.
              NX is built on top of NXGLIB.

Related Header Files
^^^^^^^^^^^^^^^^^^^^

include/nuttx/nxglib.h    -- Describes the NXGLIB C interfaces
include/nuttx/nx.h        -- Describes the NX C interfaces
include/nutt/nxtk.h       -- Describe the NXTOOLKIT C interfaces
include/nutt/nxfont.h     -- Describe sthe NXFONT C interfaces
include/nuttx/nxwidgets.h -- Will describe the NXWIDGETS classes (no longer planned)

Directories:
^^^^^^^^^^^^

graphics/nxglib
  The NuttX tiny graphics library.  The directory contains generic utilities
  support operations on primitive graphics objects and logic to rasterize directly
  into a framebuffer.  It has no concept of windows (other than the one, framebuffer
  window).

graphics/nxbe
  This is the "back-end" of a tiny windowing system.  It can be used with either of
  two front-ends to complete a windowing system (see nxmu and nxsu below).  It
  contains most of the important window management logic:  clipping, window controls,
  window drawing, etc.

graphics/nxsu
  This is the NX single user "front end".  When combined with the generic "back-end"
  (nxbe), it implements a single thread, single user windowing system.  The files
  in this directory present the window APIs described in include/nuttx/nx.h.  The
  single user front-end is selected when CONFIG_NX_MULTIUSER is not defined in the
  NuttX configuration file.

graphics/nxmu
  This is the NX multi user "front end".  When combined with the generic "back-end"
  (nxbe), it implements a multi-threaded, multi-user windowing system.  The files
  in this directory present the window APIs described in include/nuttx/nx.h.  The
  multi-user front end includes a graphics server that executes on its own thread;
  multiple graphics clients then communicate with the server via a POSIX message
  queue to serialize window operations from many threads. The multi-user front-end
  is selected when CONFIG_NX_MULTIUSER is defined in the NuttX configuration file.

graphics/nxfonts
  This is where the NXFONTS implementation resides.  This is a relatively low-
  level set of charset set/glyph management APIs.  See include/nuttx/nxfonts.h

graphics/nxtk
  This is where the NXTOOLKIT implementation resides.  This toolkit is built on
  top of NX and works with either the single-user or multi-user NX version. See
  include/nuttx/nxtk.h

graphics/nxwidgets
  At one time, I planned to put NXWIDGETS implementation here, but not anymore.

Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^

CONFIG_NX
  Enables overall support for graphics library and NX
CONFIG_NX_MULTIUSER
  Configures NX in multi-user mode
CONFIG_NX_NPLANES
  Some YUV color formats requires support for multiple planes, one for each
  color component.  Unless you have such special hardware, this value should be
  undefined or set to 1.
CONFIG_NX_DISABLE_1BPP, CONFIG_NX_DISABLE_2BPP,
CONFIG_NX_DISABLE_4BPP, CONFIG_NX_DISABLE_8BPP,
CONFIG_NX_DISABLE_16BPP, CONFIG_NX_DISABLE_24BPP, and
CONFIG_NX_DISABLE_32BPP
  NX supports a variety of pixel depths.  You can save some memory by disabling
  support for unused color depths.
CONFIG_NX_PACKEDMSFIRST
  If a pixel depth of less than 8-bits is used, then NX needs to know if the
  pixels pack from the MS to LS or from LS to MS
CONFIG_NX_MOUSE
  Build in support for mouse input.
CONFIG_NX_KBD
  Build in support of keypad/keyboard input.
CONFIG_NXTK_BORDERWIDTH
  Specifies with with of the border (in pixels) used with framed windows.
  The default is 4.
CONFIG_NXTK_BORDERCOLOR1 and CONFIG_NXTK_BORDERCOLOR2
  Specify the colors of the border used with framed windows.
  CONFIG_NXTK_BORDERCOLOR2 is the shadow side color and so is normally darker.
  The default is medium and dark grey, respectively
CONFIG_NXTK_AUTORAISE
  If set, a window will be raised to the top if the mouse position is over a
  visible portion of the window.  Default: A mouse button must be clicked over
  a visible portion of the window.
CONFIG_NXFONTS_CHARBITS
  The number of bits in the character set.  Current options are only 7 and 8.
  The default is 7.
CONFIG_NXFONT_SANS23X27
  At present, there is only one font, a 23x27 sans serif fount.  But if
  there were were more, then this option would select that sans serif font.

NX Multi-user only options:

CONFIG_NX_BLOCKING
  Open the client message queues in blocking mode.  In this case,
  nx_eventhandler() will not return until a message is received and processed.
CONFIG_NX_MXSERVERMSGS and CONFIG_NX_MXCLIENTMSGS
  Specifies the maximum number of messages that can fit in the message queues.
  No additional resources are allocated, but this can be set to prevent
  flooding of the client or server with too many messages (CONFIG_PREALLOC_MQ_MSGS
  controls how many messages are pre-allocated).


