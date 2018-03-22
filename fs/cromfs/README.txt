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

Architecture
============

The CROMFS file system is represented by an in-memory data structure.  This
structure is a tree.  At the root of the tree is a "volume" node that
describes the overall operating system.  Other entities within the file
system are presented by other nodes:  hard links, directories, and files.
These nodes are all described in fs/cromfs/cromfs.h.

In addition to general volume information, the volume node provides an
offset to the the "root directory".  The root directory, like all other
CROMFS directories is simply a singly linked list of other nodes:  hard link
nodes, directory nodes, and files.  This list is managed by a "peer
offsets":  Each node in the directory contains an offset to its peer in the
same directory.  This directory list is terminated with a zero offset.

Hard link, directory, and file nodes all include such a "peer offset".  Hard
link nodes simply refer to other others and are more or less contained.
Directory nodes contain, in addition, a "child offset" to the first entry in
another singly linked list of nodes comprising the directory.

File nodes provide file data.  They are followed by a variable length list
of compressed data blocks.  Each compressed data block contains an LZF
header as described in include/lzf.h.

So, given this information, we could illustrate the sample CROMFS file
system above with these nodes (where V=volume node, H=Hard link node,
D=directory node, F=file node, D=Data block:

  V
  `- +- H: .
     |
     +- F: BaaBaaBlackSheep.txt
     |  `- D,D,D,...D
     +- D: emptydir
     |  |- H: .
     |  `- H: ..
     +- F: JackSprat.txt
     |  `- D,D,D,...D
     +- D: testdir1
     |  |- H: .
     |  |- H: ..
     |  |- F: DingDongDell.txt
     |  |  `- D,D,D,...D
     |  `- F: SeeSawMargorieDaw.txt
     |     `- D,D,D,...D
     +- D: testdir2
     |  |- H: .
     |  |- H: ..
     |  |- F: HickoryDickoryDock.txt
     |  |  `- D,D,D,...D
     |  `- F: TheThreeLittlePigs.txt
     |     `- D,D,D,...D
     +- D: testdir3
        |- H: .
        |- H: ..
        `- F: JackBeNimble.txt
           `- D,D,D,...D

Configuration
=============

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
