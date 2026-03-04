=============
``indent.sh``
=============

This script can be used to indent .c and .h files in a manner similar
to the NuttX coding style.  It doesn't do a really good job, however
(see below and the comments at the top of the indent.sh file).

USAGE::

    tools/indent.sh [-d] [-p] -o <out-file> <in-file>
    tools/indent.sh [-d] [-p] <in-file-list>
    tools/indent.sh [-d] -h

Where::

    -<in-file>
      A single, unformatted input file
    -<in-file-list>
      A list of unformatted input files that will be reformatted in place.
    -o <out-file>
      Write the single, reformatted <in-file> to <out-file>.  <in-file>
      will not be modified.
    -d
      Enable script debug
    -p
      Comments are pre-formatted.  Do not reformat.
    -h
      Show this help message and exit

The conversions make by the indent.sh script differs from the NuttX coding
style in that:

1. The coding standard requires that the trailing ``*/`` of a multi-line
   comment be on a separate line.  By default, indent.sh will put the
   final ``*/`` on the same line as the last comment text.  If your C file
   already has properly formatted comments then using the ``-p`` option will
   eliminate that bad behavior

2. If your source file has highly formatted comments containing things
   such as tables or lists, then use the -p option to preserve those
   pre-formatted comments.

3. I usually align things vertically (like '=' in assignments),

4. indent.sh puts a bogus blank line at the top of the file,

5. I don't like the way it handles nested conditional compilation
   intermixed with code.  I prefer the preprocessor conditional tests
   be all right justified in that case.

6. I also indent brackets differently on structures than does this script.

7. I normally use no spaces in casts.  indent.sh adds spaces in casts like
   ``(FAR void *)&foo`` becomes ``(FAR void *) & foo``.

8. When used with header files, the initial idempotence conditional test
   causes all preprocessor directives to be indented in the file.  So for
   header files, you will need to substitute "^#  " with "#" in the
   converted header file.

You will manually need to check for the issues listed above after
performing the conversions.  nxstyle.c provides a good test that will
catch most of the indent.sh screw-ups.  Together, they do a pretty good
job of formatting.

See also nxstyle.c and uncrustify.cfg

