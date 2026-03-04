==================
``kconfig2html.c``
==================

This is a C file that can be used to build a utility for converting the
NuttX configuration in the Kconfig files to an HTML document.  This
auto-generated documentation will, eventually, replace the manually
updated configuration documentation that is falling woefully behind:

.. code:: console

   $ tools/kconfig2html.exe -h
   USAGE: tools/kconfig2html [-d] [-a <apps directory>] {-o <out file>] [<Kconfig root>]
          tools/kconfig2html [-h]

Where::

    -a : Select relative path to the apps/ directory. This path is relative
         to the <Kconfig directory>.  Default: ../apps
    -o : Send output to <out file>.  Default: Output goes to stdout
    -d : Enable debug output
    -h : Prints this message and exits
    <Kconfig root> is the directory containing the root Kconfig file.
         Default <Kconfig directory>: .


.. note::

   In order to use this tool, some configuration must be in-place with
   all necessary symbolic links.  You can establish the configured symbolic
   links with::

       make context

   or more quickly with::

       make .dirlinks
