===========================
``inifile`` INI File Parser
===========================

Syntax
------

This directory contains a very simple INI file parser. An INI file consists of a
sequence of lines up to the end of file. A line may be one of the following:

1. A blank line.

2. A comment line. Any line beginning with ``;``

3. A section header. Definitions are divided into sections. Each section begins
   with a line containing the section name enclosed in square brackets. For
   example, ``[section1]``. The left bracket must be the first character on the
   line. Section names are case insensitive, i.e., ``SECTION1`` and ``Section1``
   refer to the same section.

4. Variable assignments. A variable assignment is a variable name followed by
   the ``=`` sign and then the value of the variable. For example, ``A=B``: ``A`` is
   the variable name; ``B`` is the variable value. All variables following the
   section header belong in the section.

   Variable names may be preceded with white space. Whitespace is not permitted
   before the ``=`` sign. Variable names are case insensitive, i.e., ``A`` and ``a``
   refer to the same variable name.

   Variable values may be numeric (any base) or a string. The case of string
   arguments is preserved.

Programming Interfaces
----------------------

See ``apps/include/fsutils/inifile.h`` for interfaces supported by the INI file
parser.

## Test Program

Below is a simple test program:

.. code-block:: C

  int main(int argc, char *argv[])
    {
      INIHANDLE handle;
      FILE *stream;
      FAR char *ptr;
      long value;

      stream = fopen("/tmp/file.ini", "w");
      fprintf(stream, "; Test INI file\n");
      fprintf(stream, "[section1]\n");
      fprintf(stream, "  VAR1=1\n");
      fprintf(stream, "  VAR2=2\n");
      fprintf(stream, "  VAR3=3\n");
      fprintf(stream, "\n");
      fprintf(stream, "[section2]\n");
      fprintf(stream, "  VAR4=4\n");
      fprintf(stream, "  VAR5=5\n");
      fprintf(stream,   "VAR6=6\n");
      fprintf(stream, "\n");
      fclose(stream);

      handle = inifile_initialize("/tmp/file.ini");

      ptr = inifile_read_string(handle, "section2", "VAR5", "OOPS");
      printf("Section: %s Variable: %s String: %s\n", "section2", "VAR5", ptr);
      inifile_free_string(ptr);

      ptr = inifile_read_string(handle, "section1", "VAR2", "OOPS");
      printf("Section: %s Variable: %s String: %s\n", "section1", "VAR2", ptr);
      inifile_free_string(ptr);

      ptr = inifile_read_string(handle, "section3", "VAR3", "OOPS");
      printf("Section: %s Variable: %s String: %s\n", "section3", "VAR3", ptr);
      inifile_free_string(ptr);

      ptr = inifile_read_string(handle, "section1", "VAR3", "OOPS");
      printf("Section: %s Variable: %s String: %s\n", "section1", "VAR3", ptr);
      inifile_free_string(ptr);

      ptr = inifile_read_string(handle, "section1", "VAR1", "OOPS");
      printf("Section: %s Variable: %s String: %s\n", "section1", "VAR1", ptr);
      inifile_free_string(ptr);

      ptr = inifile_read_string(handle, "section1", "VAR42", "OOPS");
      printf("Section: %s Variable: %s String: %s\n", "section1", "VAR42", ptr);
      inifile_free_string(ptr);

      ptr = inifile_read_string(handle, "section2", "VAR6", "OOPS");
      printf("Section: %s Variable: %s String: %s\n", "section2", "VAR6", ptr);
      inifile_free_string(ptr);

      ptr = inifile_read_string(handle, "section2", "VAR4", "OOPS");
      printf("Section: %s Variable: %s String: %s\n", "section2", "VAR4", ptr);
      inifile_free_string(ptr);

      value = inifile_read_integer(handle, "section1", "VAR3", 0);
      printf("Section: %s Variable: %s Value: %ld\n", "section1", "VAR3", value);

      value = inifile_read_integer(handle, "section3", "VAR3", 0);
      printf("Section: %s Variable: %s String: %ld\n", "section3", "VAR3", value);

      value = inifile_read_integer(handle, "section1", "VAR1", 0);
      printf("Section: %s Variable: %s Value: %ld\n", "section1", "VAR1", value);

      value = inifile_read_integer(handle, "section2", "VAR5", 0);
      printf("Section: %s Variable: %s Value: %ld\n", "section2", "VAR5", value);

      value = inifile_read_integer(handle, "section2", "VAR6", 0);
      printf("Section: %s Variable: %s Value: %ld\n", "section2", "VAR6", value);

      value = inifile_read_integer(handle, "section1", "VAR42", 0);
      printf("Section: %s Variable: %s String: %ld\n", "section1", "VAR42", value);

      value = inifile_read_integer(handle, "section1", "VAR2", 0);
      printf("Section: %s Variable: %s Value: %ld\n", "section1", "VAR2", value);

      value = inifile_read_integer(handle, "section2", "VAR4", 0);
      printf("Section: %s Variable: %s Value: %ld\n", "section2", "VAR4", value);

      inifile_uninitialize(handle);
      return 0;
    }

Test program output::

  Section: section2 Variable: VAR5 String: 5
  Section: section1 Variable: VAR2 String: 2
  Section: section3 Variable: VAR3 String: OOPS
  Section: section1 Variable: VAR3 String: 3
  Section: section1 Variable: VAR1 String: 1
  Section: section1 Variable: VAR42 String: OOPS
  Section: section2 Variable: VAR6 String: 6
  Section: section2 Variable: VAR4 String: 4

  Section: section1 Variable: VAR3 Value: 3
  Section: section3 Variable: VAR3 Value: 0
  Section: section1 Variable: VAR1 Value: 1
  Section: section2 Variable: VAR5 Value: 5
  Section: section2 Variable: VAR6 Value: 6
  Section: section1 Variable: VAR42 String: 0
  Section: section1 Variable: VAR2 Value: 2
  Section: section2 Variable: VAR4 Value: 4
