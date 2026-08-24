===
NFS
===

Network file system (NFS) client file system.

Limitations
===========

Advisory locks are local to this NuttX instance (no NLM/NFSv4
locking) and inotify watches report only locally initiated changes.
