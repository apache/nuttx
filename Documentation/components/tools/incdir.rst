===========================================
``incdir.sh``, ``incdir.bat``, ``incdir.c``
===========================================

Different compilers have different conventions for specifying lists
of include file paths on the compiler command line. This incdir.sh
bash script allows the build system to create include file paths without
concern for the particular compiler in use.

The incdir.bat script is a counterpart for use in the native Windows
build.  However, there is currently only one compiler supported in
that context:  MinGW-GCC.

incdir.c is a higher performance version of incdir.sh, converted to C.
