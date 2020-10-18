===========
LED Support
===========

A board architecture may or may not have LEDs. If the board does
have LEDs, then most architectures provide similar LED support
that is enabled when ``CONFIG_ARCH_LEDS`` is selected in the NuttX
configuration file. This LED support is part of
architecture-specific logic and is not managed by the core NuttX
logic. However, the support provided by each architecture is
sufficiently similar that it can be documented here.

Header Files
============

LED-related definitions are provided in two header files:

-  LED definitions are provided for each board in the ``board.h``
   that resides in the ``<board-name>/include/board.h`` file
   (which is also linked to ``include/arch/board/board.h`` when
   the RTOS is configured). Those definitions are discussed
   `below <#leddefinitions>`__.
-  The board-specific logic provides unique instances of the LED
   interfaces. This is because the implementation of LED support
   may be very different on different boards. Prototypes for these
   board-specific implementations are, however, provided in
   architecture-common header files. That header file is usually
   at ``<arch-name>/src/common/up_internal.h``, but could be at
   other locations in particular architectures. These prototypes
   are discussed `below <#ledapis>`__.

LED Definitions
===============

The implementation of LED support is very specific to a board
architecture. Some boards have several LEDS, others have only one
or two. Some have none. Others LED matrices and show alphanumeric
data, etc. The NuttX logic does not refer to specific LEDS,
rather, it refers to an event to be shown on the LEDS in whatever
manner is appropriate for the board; the way that this event is
presented depends upon the hardware available on the board.

The model used by NuttX is that the board can show 8 events
defined as follows in ``<board-name>/include/board.h``:

.. code-block:: c

  #define LED_STARTED       ??
  #define LED_HEAPALLOCATE  ??
  #define LED_IRQSENABLED   ??
  #define LED_STACKCREATED  ??
  #define LED_INIRQ         ??
  #define LED_SIGNAL        ??
  #define LED_ASSERTION     ??
  #define LED_PANIC         ??

The specific value assigned to each pre-processor variable can be
whatever makes the implementation easiest for the board logic. The
*meaning* associated with each definition is as follows:

-  ``LED_STARTED`` is the value that describes the setting of the
   LEDs when the LED logic is first initialized. This LED value is
   set but never cleared.
-  ``LED_HEAPALLOCATE`` indicates that the NuttX heap has been
   configured. This is an important place in the boot sequence
   because if the memory is configured wrong, it will probably
   crash leaving this LED setting. This LED value is set but never
   cleared.
-  ``LED_IRQSENABLED`` indicates that interrupts have been
   enabled. Again, during bring-up (or if there are hardware
   problems), it is very likely that the system may crash just
   when interrupts are enabled, leaving this setting on the LEDs.
   This LED value is set but never cleared.
-  ``LED_STACKCREATED`` is set each time a new stack is created.
   If set, it means that the system attempted to start at least
   one new thread. This LED value is set but never cleared.
-  ``LED_INIRQ`` is set and cleared on entry and exit from each
   interrupt. If interrupts are working okay, this LED will have a
   dull glow.
-  ``LED_SIGNAL`` is set and cleared on entry and exit from a
   signal handler. Signal handlers are tricky so this is
   especially useful during bring-up or a new architecture.
-  ``LED_ASSERTION`` is set if an assertion occurs.
-  ``LED_PANIC`` will blink at around 1Hz if the system panics and
   hangs.

Common LED interfaces
=====================

The ``include/nuttx/board.h`` includes the following declarations:

.. c:function:: void board_autoled_initialize(void)

  Called early in power-up initialization to initialize the LED hardware.

  .. note:: In most architectures,
    ``board_autoled_initialize()`` is called from board-specific
    initialization logic. But there are a few architectures
    where this initialization function is still called from
    common chip architecture logic. This interface is not,
    however, a common board interface in any event.

  .. warning:: This interface name will eventually be removed;
    do not use it in new board ports. New implementations should
    not use the naming convention for common board interfaces,
    but should instead use the naming conventions for
    microprocessor-specific interfaces or the board-specific
    interfaces (such as ``stm32_led_initialize()``).

.. c:function:: void board_autoled_on(int led)

  Called to instantiate the LED
  presentation of the event. The ``led`` argument is one of the
  definitions provided in ``<board-name>/include/board.h``.

.. c:function:: void board_autoled_off(int led)

  Called to terminate the LED
  presentation of the event. The ``led`` argument is one of the
  definitions provided in ``<board-name>/include/board.h``. Note
  that only ``LED_INIRQ``, ``LED_SIGNAL``, ``LED_ASSERTION``, and
  ``LED_PANIC`` indications are terminated.
