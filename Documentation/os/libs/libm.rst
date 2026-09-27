=================
Math library
=================

Which implementation of ``<math.h>`` gets linked in.  The choice matters
more than it looks: floating point code pulls in a surprising amount of
object code, and these implementations trade size against accuracy and speed
differently.

The configuration lives in ``libs/libm/``.  It is a Kconfig ``choice``, so
exactly one applies, and the default is ``CONFIG_LIBM_TOOLCHAIN`` -- or
``CONFIG_LIBM_NONE`` when ``CONFIG_DEFAULT_SMALL`` is set.  Either way NuttX
does not build a math library of its own unless you ask it to.

The choices
===========

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Option
     - What it selects
   * - ``CONFIG_LIBM``
     - The implementation that ships with NuttX, from the Rhombus OS.  It
       also selects ``CONFIG_ARCH_FLOAT_H``.
   * - ``CONFIG_LIBM_NEWLIB``
     - Newlib's math library.
   * - ``CONFIG_LIBM_LIBMCS``
     - LibmCS.  Also needs ``CONFIG_ALLOW_BSD_COMPONENTS``, so a build that
       must avoid BSD licensed code cannot use it.
   * - ``CONFIG_LIBM_OPENLIBM``
     - OpenLibm.
   * - ``CONFIG_LIBM_TOOLCHAIN``
     - Whatever the toolchain already ships.  Nothing is built; the
       toolchain's own library is linked.
   * - ``CONFIG_LIBM_NONE``
     - No math library at all.  Code that calls ``sin()`` will fail to link,
       which is the point: on a system that should not be doing floating
       point, this makes it impossible rather than merely unwise.

The first four are the ones NuttX compiles, and all four are available only
when ``CONFIG_ARCH_MATH_H`` is *not* set -- that is, when the architecture
does not supply a ``math.h`` of its own at
``arch/<architecture>/include/math.h``.  Where it does, that one wins and
the choice narrows to the toolchain's library or none.

Choosing
========

``CONFIG_LIBM_TOOLCHAIN`` is the least surprising choice when the toolchain
has a usable library, since nothing is compiled and nothing can disagree
about representations.  ``CONFIG_LIBM`` is the portable fallback.  The other
three are there because a project may already have made this decision for
reasons of certification, licence or accuracy, and NuttX should not force a
second one.
