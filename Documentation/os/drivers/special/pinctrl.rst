======================
Pinctrl Device Drivers
======================

- The Pinctl driver framework allows applications and drivers to flexibly configure
  and manage pin parameters such as functionality, strength, driver type, and slewrate
  (voltage transition speed). This framework significantly enhances the flexibility
  and configurability of the system in terms of hardware interface control.

-  ``include/nuttx/pinctrl/pinctrl.h``
   All structures and APIs needed to work with pinctrl drivers are provided in
   this header file.

-  ``struct pinctrl_dev_s`` and ``struct pinctrl_ops_s``.
   Each pinctrl device driver must implement an instance of ``struct pinctrl_dev_s``.
   And the ``struct pinctrl_ops_s`` defines a call table with the following methods:

   #. **set_function**: Configures the pin's multiplexing (Mux) function, allowing it
      to be set as a specific hardware interface (e.g., UART, SPI, I2C) or as a
      general-purpose GPIO pin.
   #. **set_strength**: Allows the user to configure the pin's drive strength to meet
      the requirements of different hardware interfaces.
   #. **set_driver**: Controls the pin's driver type, such as push-pull output or
      open-drain output.
   #. **set_slewrate**: Enables the configuration of pin slew rate, which is crucial
      for high-speed digital signal transmission, optimizing signal rise and fall times.
   #. **select_gpio**: Configures the pin function as GPIO.
   #. **get_pad**: Reports what a pad currently holds.  Optional; see
      `Reading a pad back`_.

- Convenience macros are provided to map these operations directly:
  ``PINCTRL_SETFUNCTION``,``PINCTRL_SETSTRENGTH``,``PINCTRL_SETDRIVER``,``PINCTRL_SETSLEWRATE``,
  ``PINCTRL_SELECTGPIO``.

- Application developers can configure and control pins by opening /dev/pinctrl0 nodes
  and using the ioctl system call.
  cmd: PINCTRLC_SETFUNCTION, PINCTRLC_SETSTRENGTH, PINCTRLC_SETDRIVER, PINCTRLC_SETSLEWRATE,
  PINCTRLC_SELECTGPIO, PINCTRLC_GETPAD.
  parameters: struct pinctrl_param_s, and struct pinctrl_getpad_s for
  PINCTRLC_GETPAD.

Reading a pad back
==================

The five operations above only write.  ``get_pad`` is the one that reads:
it fills a ``struct pinctrl_padinfo_s`` describing what a pad currently
holds, and serves both the ``PINCTRLC_GETPAD`` ioctl and ``/proc/pinctrl``.

The method is **optional**.  A controller that does not implement it is
still listed in ``/proc/pinctrl``, and ``PINCTRLC_GETPAD`` returns
``-ENOTSUP``.

A pad need not have every field.  Fill only what the pad really has and
set the matching validity bit in ``have``; a field whose bit is clear
renders as ``-``:

   ============================ ==========================================
   Bit                          Field
   ============================ ==========================================
   ``PINCTRL_HAVE_FUNCTION``    ``function``
   ``PINCTRL_HAVE_STRENGTH``    ``strength``
   ``PINCTRL_HAVE_PULL``        ``pullup`` and ``pulldown``
   ``PINCTRL_HAVE_SLEWRATE``    ``slewrate``
   ``PINCTRL_HAVE_INPUT``       ``input``
   ``PINCTRL_HAVE_SCHMITT``     ``schmitt``
   ============================ ==========================================

Anything the structure has no member for goes in ``extra`` as further
``key:value`` text, which is appended to the pad's line.

``npins`` in ``struct pinctrl_dev_s`` bounds the pins ``get_pad`` is asked
about.

A controller implementing the method looks about like this:

.. code-block:: c

   static int mychip_getpad(struct pinctrl_dev_s *dev, uint32_t pin,
                            struct pinctrl_padinfo_s *info)
   {
     uint32_t val = getreg32(MYCHIP_PAD(pin));
     FAR const char *name;

     info->have     = PINCTRL_HAVE_FUNCTION | PINCTRL_HAVE_PULL;
     info->function = (val & PAD_FUNC_MASK) >> PAD_FUNC_SHIFT;
     info->pullup   = (val & PAD_PU) != 0;
     info->pulldown = (val & PAD_PD) != 0;

     /* Names are optional; both helpers return NULL when a name is
      * absent, which leaves the strings empty.
      */

     name = pinctrl_padname(g_mychip_padnames,
                            nitems(g_mychip_padnames), pin);
     if (name != NULL)
       {
         strlcpy(info->name, name, sizeof(info->name));
       }

     name = pinctrl_funcname(g_mychip_padnames,
                             nitems(g_mychip_padnames), pin,
                             info->function);
     if (name != NULL)
       {
         strlcpy(info->funcname, name, sizeof(info->funcname));
       }

     /* Anything the structure has no member for */

     snprintf(info->extra, sizeof(info->extra), "ms:%u",
              (val >> 8) & 3);
     return OK;
   }

   static const struct pinctrl_ops_s g_mychip_ops =
   {
     ...
     .get_pad = mychip_getpad,
   };

Naming pads and functions
-------------------------

The ``name`` and ``funcname`` members are **entirely optional**: leave
them empty and the pad is reported by number alone.  A controller that
wants names may declare them, one entry per pad, with
``PINCTRL_PADNAME()`` giving the pad's name followed by the name of each
function select in select order.  ``NULL`` marks a select the hardware
documentation does not name:

.. code-block:: c

   static const struct pinctrl_padname_s g_mychip_padnames[] =
   {
     [MYCHIP_PAD_I2C0_SCL] = PINCTRL_PADNAME("I2C0_SCL", "I2C0_SCL",
                                             NULL, "GPIO44"),
     [MYCHIP_PAD_SPI0_CLK] = PINCTRL_PADNAME("SPI0_CLK", "SPI0_CLK"),
     [MYCHIP_PAD_XIN]      = PINCTRL_PADNAME("XIN", NULL),
   };

The designated initializers make the array index the pad id, which is what
the lookups assume, and a pad left out of the table reports as a number.
``I2C0_SCL`` above has three selects with the middle one undocumented,
``SPI0_CLK`` has one, and ``XIN`` has a pad name but no named function.

``pinctrl_padname()`` and ``pinctrl_funcname()`` look up that table and
return ``NULL`` when a name is absent, so a controller can pass their
results straight through.  Both take the table's length and bound the pin
against it, so a pin beyond the table returns ``NULL`` rather than reading
past the end.

Since the names exist only to be printed, guard the table with the
configuration that prints them and let the lookups return ``NULL``
otherwise; the output falls back to numbers.

/proc/pinctrl
=============

``CONFIG_PINCTRL_PROCFS`` adds ``/proc/pinctrl``, which lists every
registered controller and, for those implementing ``get_pad``, one line
per pad.  Every line carries the same tokens in the same order, so the
file can be parsed:

.. code-block:: text

   pinctrl0: 166 pads
   0    CHIP_MODE            func:0 sel:CHIP_MODE        ds:0 pu:0 pd:1 ie:1 smt:1 slew:-
   91   I2C0_SCL             func:0 sel:I2C0_SCL         ds:1 pu:0 pd:0 ie:1 smt:0 slew:-
   164  ADDR_RGMII0_SEL_MODE func:- sel:-                ds:- pu:- pd:- ie:- smt:- slew:- ms1:1 ms2:1

The option depends on ``FS_PROCFS_REGISTER`` and is off by default.

