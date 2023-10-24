===============================
``ficl`` Ficl Forth interpreter
===============================

Ficl is a programming language interpreter designed to be embedded into other
systems as a command, macro, and development prototyping language.

This is DIY port of Ficl (the "Forth Inspired Command Language"). See
http://ficl.sourceforge.net/. It is a "" port because the Ficl source is not
in that directory, only an environment and instructions that will let you build
Ficl under NuttX. The rest is up to you.

Build Instructions
------------------

Disclaimer: This installation steps have only been exercised using Ficl 4.1.0.
With new versions you will likely have to make some adjustments to this
instructtions or to the files within this directory. Think of this information
as _recommendations_ - not necessarily proven instructions.

1. ``cd`` to ``interpreters/ficl``

2. Download Ficl: http://sourceforge.net/projects/ficl/files/

3. Uznip the Ficl compressed file.

   For example, ``unzip ficl-4.1.0.zip`` will leave the file
   ``interpreters/ficl/ficl-4.1.0``.

4. Configure to build Ficl in the ``interpreters/ficl`` directory using the
   ``configure.sh`` script.

   For example, ``./configure.sh ficl-4.1.0`` will leave the Makefile fragment
   ``Make.srcs`` in the ficl build directory.

5. Create your NuttX configuration. Using the ``make menuconfig``, you should
   select::

     CONFIG_INTERPRETERS_FICL=y

6. Configure and build NuttX. On successful completion, the Ficl objects will be
   available in ``apps/libapps.a`` and that NuttX binary will be linked against
   that file. Of course, Ficl will do nothing unless you have written some
   application code that uses it!
