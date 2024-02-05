========
Examples
========

Selecting Examples
------------------

The examples directory contains several sample applications that can be linked
with NuttX. The specific example is selected in the
``boards/<arch-name>/<chip-name>/<board-name>/configs/<config>/defconfig`` file
via the ``CONFIG_EXAMPLES_xyz`` setting where ``xyz`` is the name of the example.
For example::

  CONFIG_EXAMPLES_HELLO=y

Selects the ``examples/hello`` "Hello, World!" example.

Built-In Functions
------------------

Some of the examples may be built as built-in functions that can be executed
at run time (rather than as NuttX main programs). These built-in examples
can be also be executed from the NuttShell (NSH) command line. In order to
configure these built-in NSH functions, you have to set up the following:

- ``CONFIG_NSH_BUILTIN_APPS`` – Enable support for external registered, named
  applications that can be executed from the NSH command line (see
  ``apps/README.md`` for more information).


Supported examples
------------------

.. toctree::
   :glob:
   :maxdepth: 1
   :titlesonly:
   :caption: Contents
   
   */index*
