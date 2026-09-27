=========================
Build System and Tooling
=========================

Reference for the machinery around the OS rather than the OS itself: the
CMake build, the linker facility that lets code register itself at link time,
what a board directory holds and how to add one, and the programs that run on
your host.  The subsystems that make up NuttX are in :doc:`/os/index`, and
how to compile and configure a build in the first place is in
:doc:`/quickstart/index`.

One word on ``boards/``, because it can look like an exception: the code
there *is* part of the built image, as :doc:`boards` says in its opening
lines.  What this section documents is the configuration side -- what a board
directory holds, how to configure NuttX for one, and how to add another.

.. toctree::
   :maxdepth: 2

   cmake.rst
   iterable_sections.rst
   boards.rst
   tools/index.rst
