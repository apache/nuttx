README
======

This directory contains the the CROMFS file system.  This is an in-memory
(meaning no block driver), read-only (meaning that can lie in FLASH) file
system.  It uses LZF decompression on data only (meta data is not
compressed).

It accesses the in-memory file system via directory memory reads and, hence,
can only reside in random access NOR-like FLASH.  It is intended for use
with on-chip FLASH available on most MCUs (the design could probably be
extended to access non-random-access FLASH as well, but those extensions
are not yet in place).

I do not have a good way to measure how much compression we get use LZF.  I
have seen 37% compression reported in other applications, so I have to
accept that for now.  That means, for example, that you could have a file
system with 512Kb of data in only 322Kb of FLASH, giving you 190Kb to do
 other things with.

LZF compression is not known for its high compression rations, but rather
for fast decompression.  According to the author of the LZF decompression
routine, it is nearly as fast as a memcpy!

There is also a new tool at /tools/gencromfs.c that will generate binary
images for the NuttX CROMFS file system and and an example CROMFS file
system image at apps/examples/cromfs.  That example includes a test file
system that looks like:

  $ ls -Rl ../apps/examples/cromfs/cromfs
  ../apps/examples/cromfs/cromfs:
  total 2
  -rwxr--r--+ 1 spuda spuda 171 Mar 20 08:02 BaaBaaBlackSheep.txt
  drwxrwxr-x+ 1 spuda spuda   0 Mar 20 08:11 emptydir
  -rwxr--r--+ 1 spuda spuda 118 Mar 20 08:05 JackSprat.txt
  drwxrwxr-x+ 1 spuda spuda   0 Mar 20 08:06 testdir1
  drwxrwxr-x+ 1 spuda spuda   0 Mar 20 08:10 testdir2
  drwxrwxr-x+ 1 spuda spuda   0 Mar 20 08:05 testdir3
  ../apps/examples/cromfs/cromfs/emptydir:
  total 0
  ../apps/examples/cromfs/cromfs/testdir1:
  total 2
  -rwxr--r--+ 1 spuda spuda 249 Mar 20 08:03 DingDongDell.txt
  -rwxr--r--+ 1 spuda spuda 247 Mar 20 08:06 SeeSawMargorieDaw.txt
  ../apps/examples/cromfs/cromfs/testdir2:
  total 5
  -rwxr--r--+ 1 spuda spuda  118 Mar 20 08:04 HickoryDickoryDock.txt
  -rwxr--r--+ 1 spuda spuda 2082 Mar 20 08:10 TheThreeLittlePigs.txt
  ../apps/examples/cromfs/cromfs/testdir3:
  total 1
  -rwxr--r--+ 1 spuda spuda 138 Mar 20 08:05 JackBeNimble.txt

When built into NuttX and deployed on a target, it looks like:

  NuttShell (NSH) NuttX-7.24
  nsh> mount -t cromfs /mnt/cromfs
  nsh> ls -Rl /mnt/cromfs
  /mnt/cromfs:
   dr-xr-xr-x       0 .
   -rwxr--r--     171 BaaBaaBlackSheep.txt
   dr-xr-xr-x       0 emptydir/
   -rwxr--r--     118 JackSprat.txt
   dr-xr-xr-x       0 testdir1/
   dr-xr-xr-x       0 testdir2/
   dr-xr-xr-x       0 testdir3/
  /mnt/cromfs/emptydir:
   drwxrwxr-x       0 .
   dr-xr-xr-x       0 ..
  /mnt/cromfs/testdir1:
   drwxrwxr-x       0 .
   dr-xr-xr-x       0 ..
   -rwxr--r--     249 DingDongDell.txt
   -rwxr--r--     247 SeeSawMargorieDaw.txt
  /mnt/cromfs/testdir2:
   drwxrwxr-x       0 .
   dr-xr-xr-x       0 ..
   -rwxr--r--     118 HickoryDickoryDock.txt
   -rwxr--r--    2082 TheThreeLittlePigs.txt
  /mnt/cromfs/testdir3:
   drwxrwxr-x       0 .
   dr-xr-xr-x       0 ..
   -rwxr--r--     138 JackBeNimble.txt
  nsh>

Everything I have tried works:  examining directories, catting files, etc.
The "." and ".." hard links also work:

  nsh> cd /mnt/cromfs
  nsh> cat emptydir/../testdir1/DingDongDell.txt
  Ding, dong, bell,
  Pussy's in the well.
  Who put her in?
  Little Johnny Green.

  Who pulled her out?
  Little Tommy Stout.
  What a naughty boy was that,
  To try to drown poor pussy cat,
  Who never did him any harm,
  And killed the mice in his father's barn.

  nsh>

To build the CROMFS file system, you would add the following to your
configuration:

1. Enable LZF

   CONFIG_LIBC_LZF=y
   CONFIG_LIBC_LZF_ALIGN=y
   CONFIG_LIBC_LZF_HLOG=13
   CONFIG_LIBC_LZF_SMALL=y

2. Enable the CROMFS file system:

   CONFIG_FS_CROMFS=y

3. Enable the apps/examples/cromfs example:

   CONFIG_EXAMPLES_CROMFS=y
