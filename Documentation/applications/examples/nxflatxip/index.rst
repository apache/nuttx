=================================================
``nxflatxip`` NXFLAT Executed In Place from XIPFS
=================================================

Writes an NXFLAT module into a :doc:`XIPFS </components/filesystem/xipfs>`
volume at run time, the way a download would, and runs two instances of it
concurrently. The module's text is executed directly out of flash and shared
between the instances; each instance gets its own data.

The same module in a ROMFS image would execute in place just as well. What it
could not do is arrive after the firmware was built, which is the point of
the exercise.

Usage::

    nxflatxip [bench | defrag]

With no argument it stages the module, reports the extent it landed in and
the flash address the file system will map it at, then runs both instances::

    nsh> nxflatxip
    === NXFLAT module executed in place from xipfs ===

    staged 268 bytes to /mnt/xipfs/xipmod
    extent: block 6 x1, size 268, flash addr 0x10106000

    running two concurrent instances...

    [while running] pin count on the shared text = 2

      instance seed 1: text 0x10106070, stack 0x20006860, sum 2080 -- data private and intact
      instance seed 2: text 0x10106070, stack 0x200074b0, sum 4160 -- data private and intact
    [after exit]    pin count on the shared text = 2 (released when this task exits)

The two instances report the same text address, inside the flash window and
equal to where the file lies on the media, and different stack addresses.
While they run the extent carries one pin per instance, which is what stops
the defragmenter relocating code that is executing.

The pins outlive the instances: ``binfmt`` maps the text in the context of
whoever called ``exec()``, so the mappings belong to the demo's address space
and are released when it exits. Run ``xipfs -n`` afterwards to see the count
back at zero.

Subcommands
===========

``bench``
  Times a bulk read out of the mapped flash before and after a write. On a
  target whose driver has to tear down and restore execute-in-place around
  each erase, the difference is the cost of whichever restore path it used.
  Needs a large file to read; the command says how to create one.

``defrag``
  Fills the volume, deletes every other file until an allocation genuinely
  cannot be satisfied, compacts, retries the same allocation, then verifies
  every relocated file byte for byte and again after a remount. Prints a
  block map at each step.

Building the module
===================

The module is built from ``module/xipmod.c`` at build time, exactly the way
:doc:`../nxflat/index` builds its test programs, so it needs the same two
host tools from the NuttX toolchain, ``mknxflat`` and ``ldnxflat``, and the
board's ``Make.defs`` must name them.

The module has no static data and no string constants, and reports through a
callback into the firmware rather than formatting its own output. The comment
at the top of ``module/xipmod.c`` records why: the ``ldnxflat`` in
circulation miscomputes both of those cases. The callback also exercises the
other direction of the interface, firmware code entered with the module's
data base still live in its PIC register.

Configuration
=============

``CONFIG_EXAMPLES_NXFLATXIP``
  Enable the example. Requires ``CONFIG_FS_XIPFS``, ``CONFIG_NXFLAT`` and
  ``CONFIG_LIBC_EXECFUNCS``.

``CONFIG_EXAMPLES_NXFLATXIP_MOUNTPT``
  The XIPFS volume to use.

``CONFIG_EXAMPLES_NXFLATXIP_MTD``
  The MTD device backing that volume, used only by the ``defrag``
  subcommand, which remounts to show that what it did survives.
