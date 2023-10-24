===================================
``twm4nx`` Tab Window Manager (TWM)
===================================

Twm4Nx is a port of twm, Tab Window Manager (or Tom's Window Manager) version
``1.0.10`` to NuttX NX windows server. No, a port is not the right word. It is a
re-design of TWM from the inside out to work with the NuttX NX server. The name
Twm4Nx reflects this legacy. But Twm4Nx is more a homage to TWM than a port of
TWM.

The original TWM was based on X11 which provides a rich set of features. TWM
provided titlebars, shaped windows, several forms of icon management,
user-defined macro functions, click-to-type and pointer-driven keyboard focus,
graphic contexts, and user-specified key and pointer button bindings, etc.

Twm4Nx, on the other hand is based on the NuttX NX server which provides
comparatively minimal support. Additional drawing support comes from the NuttX
NxWidgets library (which necessitated the change to C++).

Twm4Nx is greatly stripped down and targeted on small embedded systems with
minimal resources. For example, no assumption is made about the availability of
a file system; no ``.twmrc`` file is used. Bitmaps are not used (other than for
fonts).

The TWM license is, I believe compatible with the BSD license used by NuttX. The
origin TWM license required notice of copyrights within each file and a full
copy of the original license which you can find in the ``COPYING`` file. within
this directory.

Status
------

Progress
~~~~~~~~

- ``2019-04-28`` This port was brutal. Much TWM logic was removed because it
  depended on X11 features (or just because I could not understand how to use
  it). The replacement logic is only mostly in place but more needs to be done
  to have a complete system (hence, it is marked ``EXPERIMENTAL``). The kinds of
  things that need to done are:

  1. Right click should bring up a window list (like the icon manager???)
  2. For TWM-like behavior, a window frame and toolbar should be highlighted
      when the window has focus.
  3. A right click on the toolbar should bring up a window-specific menu.

- ``2019-05-02`` Some testing progress. The system comes up, connects to and
  initializes the VNC window. For some reason, the VNC client breaks the
  connection. The server is no longer connected so Twm4Nx constipates and and
  eventually hangs.

- ``2019-05-08`` I abandoned the VNC interface and found that things are much
  better using a direct, hardware framebuffer. The background comes up properly
  and the Icon Manager appears properly in the upper right hand corner. The Icon
  Manager Window can be iconified or de-iconified. The Icon Manager window can
  be grabbed by the toolbar title and moved about on the window (the movement is
  not very smooth on the particular hardware that I am working with).

- ``2019-05-10`` A left click on the background brings up the main menu. At
  present there are only two options: _Desktop_ which will iconify all windows
  and "Twm4Nx Icon Manager" which will de-iconify and/or raise the Icon Manager
  window to the top of the hierarchy. That latter option is only meaningful when
  the desktop is very crowded.

- ``2019-05-13`` Added the NxTerm application. If enabled via
  ``CONFIG_TWM4XN_NXTERM``, there will now be a _NuttShell_ entry in the Main
  Menu. When pressed, this will bring up an NSH session in a Twm4Nx window.

- ``2019-05-14`` We can now move an icon on the desktop. Includes logic to avoid
  collisions with other icons and with the background image. That later is an
  issue. The background image image widget needs to be removed; it can occlude a
  desktop icon. We need to paint the image directly on the background without
  the use of a widget.

- ``2019-05-15`` Resizing now seems to work correctly in Twm4Nx.

- ``2019-05-20`` Calibration screen is now in place.

- ``2019-05-21`` A ``CONTEMPORARY`` theme was added. Still has a few glitches.

- ``2019-06-01`` A retro, emulated segment LCD clock is now in place.

How To
------

Icon Manager
~~~~~~~~~~~~

- At start up, only the Icon Manager window is shown. The Icon Manager is a TWM
  alternative to more common desktop icons. Currently Twm4Nx supports both
  desktop icons and the Icon Manager.

  Whenever a new application is started from the Main Menu, its name shows up in
  the Icon Manager. Selecting the name will either de-iconify the window, or
  just raise it to the top of the display.

Main Menu
~~~~~~~~~

- A touch/click at any open location on the background (except the image at the
  center or on another icon) will bring up the Main Menu. Options:

  - Desktop. Iconify all windows and show the desktop
  - Twm4Nx Icom Manager. De-iconify and/or raise the Icon Manager to the top of
    the display.
  - Calibration. Perform touchscreen re-calibration.
  - Clock. Start and instance of clock in the window. The uses the the retro,
    LCD emulation of ``apps/graphics/slcd``.
  - NuttShell. Start and instance of NSH running in an NxTerm.

- All windows close after the terminal menu option is selected.

Window Toolbar
~~~~~~~~~~~~~~

- Most windows have a toolbar at the top. It is optional but used in most
  windows.
- The toolbar contains window title and from zero to 4 buttons:

  - Right side: A menu button may be presented. The menu button is not used by
    anything in the current implementation and is always suppressed
  - Left side: The far left is (1) the terminate button (if present). If
    present, it will close window when selected. Not all windows can be closed.
    You can't close the Icon Manager or menu windows, for example. Then (2) a
    resize button. If presented and is selected, then the resize sequence
    described below it started. This may the be preceded by a minimize button
    that iconifies the window.

Moving a Window
~~~~~~~~~~~~~~~

- Grab the title in the toolbar and move the window to the desired position.

Resizing a Window
~~~~~~~~~~~~~~~~~

- A window must have the green resize button with the square or it cannot be
  resized.
- Press resize button. A small window should pop-up in the upper left hand
  corner showing the current window size.
- Touch anywhere in window (not the toolbar) and slide your finger. The resize
  window will show the new size but there will be no other update to the
  display. It is thought that continuous size updates would overwhelm lower end
  MCUs. Movements support include:

  - Move toward the right increases the width of the window
  - Move toward the left decreases the width of the window
  - Move toward the bottom increases the height of the window
  - Move toward the top decreases the height of the Window
  - Other moves will affect both the height and width of the window.

- **Note**: While resizing, non-critical events from all other windows are
  ignored.

Themes
~~~~~~

- There are two themes support by the configuration system:

  - ``CONFIG_TWM4NX_CLASSIC`` – Strong bordered windows with dark primary colors.
    Reminiscent of Windows 98.
  - ``CONFIG_TWM4NX_CONTEMPORARY`` – Border-less windows in pastel shades for a
    more contemporary look.

Issues
~~~~~~

``2019-05-16`` Twm4Nx is in a very complete state but only at perhaps _alpha_ in
its maturity. You should expect to see some undocumented problems. If you see
such problems and can describe a sequence to actions to reproduce the problem,
let me know and I will try to resolve the problems.

Here are all known issues and features that are missing:

TWM Compatibilities Issues:

1. Resizing works a little differently in Twm4Nx.
2. Right click should bring up a window list
3. For TWM-like behavior, a window frame and toolbar should be highlighted when
   the window has focus.
4. A right click on the toolbar should bring up a window-specific menu.

There are no near-term plans to address these compatibility issues.

Other issues/bugs. All-in-all, I would say that Twm4Nx is maturing well and is
attaining stability. That being said, there are some issues and untested
functionality that should be addressed:

1. Icon drag movement includes logic to avoid collisions with other icons and
   with the background image. That later is an issue. We need to paint the image
   directly on the background without the use of a widget.
2. There are a few color artifacts in the toolbar of the ``CONTEMPORARY`` theme.
   These look like borders are being drawn around the toolbar widgets (even
   though the are configured to be borderless).
3. Most Twm4Nx configuration settings are hard-coded in ``*_config.hxx`` header
   files. These all need to be brought out and made accessible via Kconfig files
4. I have seen some odd behavior when many NxTerm windows have been opened
   (around 15). Specifically, I see failures to start NSH in the windows so they
   come up blank. All other behaviors seem normal. Most likely, some NxTerm
   resource allocation is failing silently and leaving things in an unusable
   state. The board I am using has 128Mb of SDRAM so I can't believe that memory
   is the limiting factor. These are, however, RAM-backed windows and will use
   significant amounts of memory. The primary issue is that the number of
   windows should probably be managed in some way to assure that the end-user
   does not experience odd behaviors when resource usage is high.
5. Menus with sub-menus have not been verified. There is no use of sub- menus in
   the current code base so I expect that there are issues when, for example,
   and item from a sub-menu item: That menu and all of its antecedent menus
   should be closed.
6. There is an optional MENU button that may appear at the far left on the
   toolbar. It is not used by any window in the current code base and, hence, is
   unverified. I would expect some issues with generating and routing the MENU
   button events to applications. There are likely other unverified features.
7. X/Y input may be either via a touchscreen or a mouse. Only touchscreen input
   has been verified. There is, however, very little difference. The primary
   issue is in cursor support: Cursors are needed with a mouse. Cursor images
   also change depending on the state (like grabbing and dragging or resizing).
   There is also a possibility of using auto-raise with a mouse as well. All of
   this logic is in place, but none has been verified.
8. NxTerm windows really need to be scrollable. They are difficult to use with
   only a few lines on a small display. A related usability issue is the font
   height: The fonts report a maximum font height that results in a large line
   spacing on the display and, hence, fewer lines visible in the small window.
   This is latter issues is a problem with the fonts not Twm4Nx, however.
9. There is a trivial rounding error in the calculation of the LCD width in
   ``SLcd::CSLcd::getWidth()``. It currently truncates down. It needs to round up.
   This sometimes leaves a small, one-pixel- wide sliver on the clock display.
   This display always recovers and this only cosmetic.

Adding Twm4Nx Applications
--------------------------

Application Factories and the Main Menu
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The original TWM supported a .twmrc in which you could describe application
programs supported on the desktop. Currently no such start-up file is available
for Twm4Nx. Rather, all applications must be added via run-time interfaces. And
overview of these interfaces is provided in this paragraph.

Currently, there are only two applications developed for Twm4Nx: (1) An NxTerm
hosting NSH that is analogous to an XTerm hosting the Bash shell in a Unix
environment, and (2) a touchscreen calibration application. Let's focus instead
on the NxTerm application as an example because the touchscreen calibration is a
rather unusual beast.

These example applications can be found at: ``apps/graphics/twm4nx/apps`` and
``apps/include/graphics/twm4nx/apps``

In short, adding an application involves a "Factory Object" that is hooked into
the Main Menu. A Factory Object is an object that is used to create other object
instances. The way in which the Factory Object is represented is purely a
decision of the application developer. One option, however, is to use the pure
virtual base class ``IApplicationFactory`` as defined in
``apps/include/graphics/twm4nx/iapplication.hxx``. This base class provides only a
single method::

  bool initialize(FAR CTwm4Nx *twm4nx);

where CTwm4Nx is the Twm4NX session instance that allows the class
implementation to interact with session specific resources. Multiple sessions
would be required, for example, if the platform supported multiple displays.

In practice, the application factory implementation class inherits from the
following base classes:

1. ``IApplicationFactory``. Provides the common ``initialize()`` method.
2. ``IApplication``. Provides the information for the application's entry in the
    Main Menu
3. ``CTwm4NxEvent``. Hooks the application factory into the Twm4Nx event
    notification system.

Initialization consists of instantiating the application factory instance and
calling its ``IApplicationFactory::initialize()`` method. The application factory
instance is a singleton that must persist for the life of the session. For the
NxTerm application factory, this is done in
``apps/graphics/twm4nx/src/twm4nx_main.c`` like::

  CNxTermFactory nxtermFactory;
  success = nxtermFactory.initialize(twm4nx);

In addition to general initialization, the ``IApplicationFactory::initialize()``
method must register a new entry with the main menu. You can see an example of
this in ``apps/graphics/twm4nx/apps/cnxterm.c``::

  FAR CMainMenu *cmain = twm4nx->getMainMenu();
  return cmain->addApplication(this);

The argument to the ``CMainMenu::addApplication()`` method is of type
``IApplication *``. Remember, however, that our application implementation ``class``
inherited from ``IApplication``.

The IApplication pure virtual base class is also defined in
``apps/include/graphics/twm4nx/iapplication.hxx``. It essentially describes what
the Main Menu logic should do when the menu item is selected. It includes these
methods:

1. ``getName()``. Provides the name string that will be shown in the Main Menu for
    this selection.
2. ``getSubMenu()``. One possibility is that selecting the Main Menu item is that
    it may bring up yet another sub-menu of options.
3. ``getEventHandler()``. Returns an instance of ``CTwm4NxEvent`` that is used to
    route menu selection events. Remember that our application factory inherits
    from ``CTwm4NxEvent`` so this function only needs to return the 'this'
    pointer.
4. ``getEvent()``. Provides the event ID that will be used in the event
    notification. The returned value must conform to the description in
    ``apps/include/graphics/twm4nx/twm4nx_events.hxx``. In particular, the
    recipient of the event must be ``EVENT_RECIPIENT_APP``.

The Twm4Nx application is then started when the application factory's
``CTwm4NxEvent::event()`` method is called with the specified event.

Application Windows
~~~~~~~~~~~~~~~~~~~

How the application factory starts an application instance is purely up to the
application designer. Typically this would include starting a new application
task. General characteristics of an application include:

1. It probably should inherit from ``CTwm4NxEvent`` so that it can receive events
    from the system.
2. To create the window, it must instantiate and initialize an instance of
    ``CWindow``.
3. It must configure application events to receive notifications from Twm4Nx.

To create an application window, the application must call the
``CWindowFactory::createWindow()`` method. For the NxTerm example, this looks
like::

  NXWidgets::CNxString name("NuttShell");

  uint8_t wflags = (WFLAGS_NO_MENU_BUTTON | WFLAGS_HIDDEN);

  FAR CWindowFactory *factory = m_twm4nx->getWindowFactory();
  m_nxtermWindow = factory->createWindow(name, &CONFIG_TWM4NX_NXTERM_ICON,
                                          (FAR CIconMgr *)0, wflags);

The window factory is another factory that creates and manages window instance.
The ``createWindow()`` method requires four parameters:

1. The name of the window. This is the name that is show in the window toolbar
    and may be the same name as was used in the Main Menu entry.
2. A reference to the the Icon image associated with the window. This is the
    image that is show on the desktop when the window is iconified. It is of
    type ``NXWidgets::SRlePaletteBitmap``.
3. A pointer to the Icon Manager instance that this window belongs with. This
    can be NULL to use the default Twm4Nx Icon Manager.
4. A set of flags that describe properties of the windows.

  The flag values are defined byte ``WFLAGS_*`` definitions provided in
  ``apps/include/graphics/twm4nx/cwindow.hxx``:

  - ``WFLAGS_NO_MENU_BUTTON`` – Omit the menu button from the toolbar.
  - ``WFLAGS_NO_DELETE_BUTTON`` – Omit the delete button from the toolbar.
  - ``WFLAGS_NO_RESIZE_BUTTON`` – Omit the resize button from the toolbar.
  - ``WFLAGS_NO_MINIMIZE_BUTTON`` – Omit the minimize button from the toolbar.
  - ``WFLAGS_NO_TOOLBAR`` – Omit the toolbar altogether.
  - ``WFLAGS_ICONMGR`` – This window is an icon manager.
  - ``WFLAGS_MENU`` – This window is a menu window.
  - ``WFLAGS_HIDDEN`` – Start with the window hidden.

Once the ``CWindow`` is instantiated, events needed by the application can be
configured as is done in the NxTerm application::

  struct SAppEvents events;
  events.eventObj    = (FAR void *)this;
  events.redrawEvent = EVENT_NXTERM_REDRAW;
  events.resizeEvent = EVENT_NXTERM_RESIZE;
  events.mouseEvent  = EVENT_NXTERM_XYINPUT;
  events.kbdEvent    = EVENT_NXTERM_KBDINPUT;
  events.closeEvent  = EVENT_NXTERM_CLOSE;
  events.deleteEvent = EVENT_NXTERM_DELETE;

  bool success = m_nxtermWindow->configureEvents(events);

Again, recall that the application inherits from ``CTwm4NxEvent``. So passing
``this`` as the event object above assures that the specific events are routed to
the application instance.

Drawing in the application window can be performed using that facilities of
NXWidgets using the ``NXWidgets::CGraphicsPort`` associated with the window. The
NxTerm application does not perform any drawing, however; that drawing is
performed by the NxTerm driver.

The ``NXWidgets::CGraphicsPort`` can be obtained from a ``CWindow`` instance, say
``m_window``, like::

  FAR NXWidgets::CWidgetControl *control = m_window->getWidgetControl();
  NXWidgets::CGraphicsPort *port = control->getGraphicsPort();

That ``CGraphicsPort`` is then passed to the widget constructor, binding the
widget to that window and forcing all widget drawing to occur within the window.

Obviously, a lot more could be written about drawing, much more than can be
addressed in this README file.
