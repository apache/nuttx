============
Reset Driver
============

Many SoCs hold their peripherals in reset until software releases them, and
share one reset line between several peripherals.  The reset framework lets a
driver ask for the line its device needs and assert or release it, without
knowing which register holds the bit or who else shares it.

A **reset controller** is the hardware block that owns the lines, described by
``struct reset_controller_dev`` and registered by an architecture or board.  A
**reset control** is one consumer's handle on one line, obtained by name or by
index and released when the consumer is done.

``include/nuttx/reset/reset.h`` declares what a consumer uses;
``include/nuttx/reset/reset-controller.h`` declares what a controller
implements.  ``CONFIG_RESET`` builds the framework.

Consumer interface
==================

A consumer obtains a handle on one line of a named controller, uses it, and
puts it back:

.. code-block:: c

   FAR struct reset_control *rstc;

   rstc = reset_control_get_exclusive_by_index("mychip-reset", 12);
   if (rstc == NULL)
     {
       return -ENODEV;
     }

   reset_control_deassert(rstc);           /* let the peripheral run */
   ...
   reset_control_put(rstc);

``reset_control_get()`` takes the controller name, the line's index, and
whether the handle is shared and already acquired.  The wrappers name the
usual combinations, and are what a driver should normally call:

   ========================================== ============================
   Function                                   Handle
   ========================================== ============================
   ``reset_control_get_exclusive()``          exclusive, acquired
   ``reset_control_get_exclusive_released()`` exclusive, not yet acquired
   ``reset_control_get_shared()``             shared
   ``reset_control_get_exclusive_by_index()`` exclusive, acquired, by index
   ``reset_control_get_shared_by_index()``    shared, by index
   ========================================== ============================

``reset_control_array_get()`` takes several lines of one controller as a
single handle, so that one call asserts or releases all of them.

The operations are:

``reset_control_assert()``
  Put the line into reset and leave it there.

``reset_control_deassert()``
  Take the line out of reset.

``reset_control_reset()``
  Pulse a self deasserting reset: the controller asserts and releases the line
  itself.  For a line that resets automatically this is the only correct
  operation; asserting such a line by hand is not meaningful.

``reset_control_status()``
  Report the line's state.  Returns 1 while the line is asserted, 0 once it is
  released, or a negated errno.  ``-ENOTSUP`` means the controller cannot read
  the line back.

``reset_control_acquire()`` and ``reset_control_release()``
  Take and give back the right to drive an exclusive line.  A handle from
  ``reset_control_get_exclusive()`` arrives already acquired; one from
  ``reset_control_get_exclusive_released()`` does not, and asserting it before
  acquiring returns ``-EPERM``.  This lets two drivers share a line in turn
  without either holding it permanently.

``reset_control_put()``
  Give the handle back.

``reset_control_device_reset()``
  Pulse the reset of a named device without holding a handle across the call.

Shared and exclusive lines
--------------------------

One line often resets several peripherals at once, and a **shared** handle is
how a driver says it is not the only user.  Shared handles are counted:
``reset_control_deassert()`` releases the line on the first call and only
counts on later ones, and ``reset_control_assert()`` asserts it only when the
last consumer has asserted.  A driver therefore gets its peripheral running
without deciding on behalf of the others when it may stop.

An **exclusive** handle assumes a single owner and drives the line
immediately.  Mixing the two on one line defeats the counting, so ask for what
the hardware really is: an exclusive handle on a shared line lets one driver
reset a peripheral another is still using.

Shared handles must not be pulsed and asserted by the same consumer:
``reset_control_reset()`` and ``reset_control_assert()`` each refuse with
``-EINVAL`` if the other has been used on that handle.

Controller interface
====================

A controller fills in ``struct reset_controller_dev`` and calls
``reset_controller_register()``:

.. code-block:: c

   static struct reset_controller_dev g_mychip_rcdev =
   {
     .name   = "mychip-reset",
     .ops    = &g_mychip_reset_ops,
     .nlines = MYCHIP_NRESETS,
   };

   reset_controller_register(&g_mychip_rcdev);

``name`` identifies the controller to ``reset_control_get()`` and heads its
section in ``/proc/reset``.  ``nlines`` bounds the ids ``get_line()`` is asked
about, which are 0 to ``nlines - 1``; since controllers commonly leave gaps it
is an id space rather than a count of real lines.  Leave it zero if the
controller does not implement ``get_line()``.  The structure must outlive registration, since the
framework stores the pointer rather than copying.

``struct reset_control_ops`` is the call table.  Every member is optional; a
consumer calling an operation the controller does not provide gets an error
rather than silence, though which error depends on the path:

   ============== =========================================================
   Method         Purpose
   ============== =========================================================
   ``reset``      Pulse a self deasserting reset on one line
   ``assert``     Put one line into reset
   ``deassert``   Take one line out of reset
   ``status``     Report whether one line is asserted; 1 asserted, 0 not
   ``acquire``    Claim a shared line
   ``release``    Give a shared line back
   ``get_line``   Describe one line for ``/proc/reset``
   ============== =========================================================

Each takes the line's ``id``, whose meaning is the controller's own: usually an
index into its own register layout, and often with gaps where an id inside the
range names no line.

``reset_controller_unregister()`` removes a controller.  Consumers must have
put their handles back first.

Reading the lines back
======================

``status()`` answers for one line at a time, and only for a caller that already
knows the id.  Nothing else in the interface says how many lines a controller
has or what any of them is for, so the framework cannot enumerate them on a
controller's behalf.

``get_line`` supplies exactly what is missing.  It is **optional**: a controller
without it is still listed in ``/proc/reset``, with a note.

.. code-block:: c

   struct reset_lineinfo_s
   {
     char name[RESET_NAME_MAX];    /* What this line resets */
     char extra[RESET_EXTRA_MAX];  /* Controller specific key:value text */
   };

Both members are optional too.  An empty name reports the line by its id alone.
``extra`` carries anything the structure has no member for, in the same
``key:value`` form, and is appended to the line's entry.

The state is **not** in the structure: the framework calls ``status()`` for it,
so a controller does not report the same fact twice.

Return ``-ENODEV`` for an id that names no line.  That is how a gap in the
numbering is reported, and the renderer skips it:

.. code-block:: c

   static int mychip_getline(FAR struct reset_controller_dev *rcdev,
                             unsigned int id,
                             FAR struct reset_lineinfo_s *info)
   {
     if (id >= nitems(g_mychip_lines) || g_mychip_lines[id].name == NULL)
       {
         return -ENODEV;              /* a gap in the numbering */
       }

     strlcpy(info->name, g_mychip_lines[id].name, sizeof(info->name));

     /* Anything the structure has no member for */

     snprintf(info->extra, sizeof(info->extra), "reg:0x%03x bit:%u",
              MYCHIP_RESET_REG(id), MYCHIP_RESET_BIT(id));
     return OK;
   }

/proc/reset
===========

``CONFIG_RESET_PROCFS`` adds ``/proc/reset``, which lists every registered
controller and, for those implementing ``get_line``, one line per reset line.
Every line carries the same tokens in the same order, so the file can be
parsed:

.. code-block:: text

   eic7700x-reset:
   0    noc_nsp                  state:released reg:0x400 bit:0
   9    gpio0                    state:held     reg:0x400 bit:9

A controller with no ``get_line`` appears as its name and a note.  ``state:-``
means the controller has no ``status()`` or the call failed.

The option depends on ``FS_PROCFS_REGISTER`` and is off by default.

Remote controllers
==================

``CONFIG_RESET_RPMSG`` lets a consumer on one core drive a reset controller
owned by another, over rpmsg.  The consumer side calls ``reset_rpmsg_get()``
for a handle to the remote controller; the core that owns the hardware runs
``reset_rpmsg_server_init()``.  Consumers use the ordinary interface either
way.
