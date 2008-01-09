README.txt
^^^^^^^^^^

The ZDS-II version 4.10.2 will not compiler NuttX.  It reports "internal
errors" on some of the files.  Upgreads to ZDS-II are available for download
from the Zilog website: http://www.zilog.com/software/zds2.asp

Thusfar, I have encountered no insolvable problems with the newer 4.11.0
version of the toolchain.

If you use any version of ZDS-II other than 4.11.0, you will have to modify
two files:  (1) configs/z16f2800100zcog/setenv.sh and (2) configs/z16f2800100zcog/Make.defs.
 
