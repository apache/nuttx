==================
``uncrustify.cfg``
==================

This is a configuration script for the uncrustify code beautifier.
Uncrustify does well with forcing braces into "if" statements and
indenting per the NuttX C coding standard. It correctly does things
like placing all braces on separate lines at the proper indentation
level.  It cannot handle certain requirements of the coding standard
such as

- FAR attributes in pointer declarations.
- The NuttX standard function header block comments.
- Naming violations such as use of CamelCase variable names,
  lower case pre-processor definitions, etc.

Comment blocks, function headers, files headers, etc. must be formatted
manually.

Its handling of block comments is fragile. If the comment is perfect,
it leaves it alone, but if the block comment is deemed to need a fix
it starts erroneously indenting the continuation lines of the comment.

- uncrustify.cfg messed up the indent of most block comments.
  cmt_sp_before_star_cont is applied inconsistently.  I added::

        cmt_indent_multi = false # disable all multi-line comment changes

  to the .cfg file to limit its damage to block comments.
- It is very strict at wrapping lines at column 78. Even when column 79
  just contained the ``/`` of a closing ``*/``.  That created many
  bad continuation lines.

- It moved '{' that opened a struct to the line defining the struct.
  nl_struct_brace = add (or force) seemed to be ignored.

- It also aligned variable names in declarations and '=' signs in
  assignment statements in a seemingly arbitrary manner. Making changes
  that were not necessary.

.. note::

    uncrustify.cfg should **ONLY** be used with new files that have an
    inconsistent coding style. uncrustify.cfg should get you in the ballpark,
    but you should expect to review and hand-edit the files to assume 100%
    compliance.

.. warning::

   **NEVER** use uncrustify.cfg for modifications to existing NuttX files. It
   will probably corrupt the style in subtle ways!

This was last verified against uncrustify 0.66.1 by Bob Feretich.

About uncrustify:  Uncrustify is a highly configurable, easily modifiable
source code beautifier.  To learn more about uncrustify:

    http://uncrustify.sourceforge.net/

Source code is available on GitHub:

    https://github.com/uncrustify/uncrustify

Binary packages are available for Linux via command line installers.
Binaries for both Windows and Linux are available at:

    https://sourceforge.net/projects/uncrustify/files/

See also :doc:`/components/tools/indent` and :doc:`/components/tools/nxstyle`.
