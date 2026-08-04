==================
Regulator Drivers
==================

A regulator is a power rail whose voltage or on/off state software can
control: a PMIC output, a switching converter, or a load switch driven by a
GPIO.  Drivers rarely want to own one outright, because a rail usually feeds
more than one peripheral, so the framework arbitrates between the consumers of
a rail rather than letting each drive it directly.

``include/nuttx/power/consumer.h`` declares what a consumer uses;
``include/nuttx/power/regulator.h`` declares what a regulator driver
implements.  ``CONFIG_REGULATOR`` builds the framework.

Consumer interface
==================

A consumer looks a regulator up by name, holds it while it needs the rail,
and puts it back:

.. code-block:: c

   FAR struct regulator_s *reg;

   reg = regulator_get("vdd-sensor");
   if (reg == NULL)
     {
       return -ENODEV;
     }

   regulator_set_voltage(reg, 1800000, 1800000);   /* microvolts */
   regulator_enable(reg);
   ...
   regulator_disable(reg);
   regulator_put(reg);

``regulator_get()``
  Look up a regulator by the name its driver registered, and return a handle
  on it.  Each handle is one consumer's claim.

``regulator_enable()`` and ``regulator_disable()``
  Ask for the rail to be on or off.  These are counted: the rail is switched
  on for the first consumer that enables it and off once the last one has
  disabled it, so a driver need not know who else is using it.

``regulator_set_voltage()``
  Ask for a voltage, as a range in microvolts.  The framework intersects the
  ranges every consumer has asked for, so a rail shared between a part that
  needs 1.8 V and one that tolerates 1.7 V to 1.9 V settles where both are
  satisfied.

``regulator_get_voltage()``
  Read the rail's present voltage, in microvolts.

``regulator_is_enabled()``
  Report whether the rail is on.

``regulator_put()``
  Give the handle back and drop this consumer's claim.

Because the framework counts consumers, a driver should hold a rail for
exactly as long as it needs it, and should not assume that disabling it turns
anything off.

Writing a regulator driver
==========================

A driver describes its regulator with a ``struct regulator_desc_s``, provides
a ``struct regulator_ops_s``, and calls ``regulator_register()``.  The
descriptor carries the rail's name, the voltage range it will accept, and how
it behaves at start up:

   ================ ========================================================
   Member           Meaning
   ================ ========================================================
   ``name``         The name consumers pass to ``regulator_get()``
   ``min_uv``       Lowest voltage the rail will accept
   ``max_uv``       Highest voltage the rail will accept
   ``supply_name``  The regulator feeding this one, if any
   ``boot_on``      The rail is expected to be on already at start up
   ``always_on``    The rail must never be switched off
   ``apply_uv``     Apply a voltage during initialisation
   ================ ========================================================

The remaining members describe the hardware to the generic helpers: the
register and mask for voltage selection and for enabling, the step size and
count for a linear rail, and the ramp and enable delays the framework waits
out after a change.

``describe`` is the one operation that exists only for reporting.  A part
often measures more than the framework has fields for, and this is where that
goes: write ``key:value`` text and the ``/proc/regulator`` renderer appends it
to the rail's line.  It is called with the list mutex held and never from
interrupt context, so reading the part over a bus is allowed.

.. code-block:: c

   static int mychip_describe(FAR struct regulator_dev_s *rdev,
                              FAR char *extra, size_t len)
   {
     FAR struct mychip_dev_s *priv = rdev->priv;

     snprintf(extra, len, "vin:%d iout:%d temp:%d",
              priv->vin_uv, priv->iout_ua, priv->temp_mc);
     return OK;
   }

The operations are all optional; a regulator with no ``is_enabled`` is treated
as always on, and one with no ``get_voltage`` reports the first entry of its
voltage list.  A GPIO controlled load switch needs only ``enable``,
``disable`` and ``is_enabled``.

``regulator_unregister()`` removes a regulator; its consumers must have put
their handles back first.

/proc/regulator
===============

The framework has no character device: consumers reach a rail from inside the
kernel, which is the right interface for controlling one but leaves no way to
see what the rails are doing.  ``CONFIG_REGULATOR_PROCFS`` adds
``/proc/regulator``, listing every registered regulator:

.. code-block:: text

   vdd-cpux    uv:900000 min:800000 max:900000 enabled:1 users:1 opens:1 supply:- always_on:1 boot_on:1
   vdd-sensor  uv:1800000 min:1700000 max:1900000 enabled:0 users:0 opens:0 supply:vsys always_on:0 boot_on:0

Every line carries the same ``key:value`` tokens in the same order, so the
file can be parsed as well as read:

   ================ ========================================================
   Token            Meaning
   ================ ========================================================
   ``uv``           Present voltage in microvolts, or ``-`` if unreadable
   ``min``, ``max`` The range the regulator will accept
   ``enabled``      Whether the rail is on, or ``-`` if unreadable
   ``users``        Consumers currently enabling it
   ``opens``        Handles currently held on it
   ``supply``       The regulator feeding this one, or ``-``
   ``always_on``    Set if the rail must never be switched off
   ``boot_on``      Set if the rail is expected on at start up
   ================ ========================================================

A driver implementing ``describe`` adds its own ``key:value`` fields after
those, so a part that measures its input voltage, output current or
temperature reports them on the same line:

.. code-block:: text

   npu_vdd    uv:800000 min:600000 max:1100000 enabled:1 users:1 opens:1 supply:- always_on:1 boot_on:0 vin:12043750 iout:3200000 temp:47500 status:0x0000

``always_on`` and ``boot_on`` are worth reading beside ``users``: a rail that
is enabled with no consumers is expected rather than suspect when either is
set.

The voltage and the enabled state are read back from the hardware rather than
recalled, so a rail the boot loader set and nothing has touched since reads as
it actually is.  For a regulator on a bus that means a transfer per rail per
read, which is why the entry takes the list mutex rather than the framework's
own lock: that one also disables interrupts, and a bus transfer waits.

The entry is read only.  What voltage a rail may be is knowledge its consumers
hold, and arranging the order between them is what the framework is for, so
moving one from a shell would step around the part that matters.

The option depends on ``FS_PROCFS_REGISTER`` and is off by default.

Remote regulators
=================

``CONFIG_REGULATOR_RPMSG`` lets a consumer on one core use a regulator owned
by another, over rpmsg.  The core that owns the hardware runs
``regulator_rpmsg_server_init()``; consumers use the ordinary interface either
way.
