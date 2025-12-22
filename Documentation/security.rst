========
Security
========

.. toctree::

Known vulnerabilities
=====================

Apache NuttX RTOS vulnerabilities are labelled with CVE (Common
Vulnerabilities and Exposures) identifiers. List of known, responsibly
disclosed, and fixed vulnerabilities are publicly available online at
`CVE.ORG <https://www.cve.org/CVERecord/SearchResults?query=nuttx>`_
and at the bottom of this page in the `NuttX CVEs`_ section.

`CVE <https://cve.org/>`_ IDs are unique identifiers given to security
vulnerabilities. The Apache Security Team is a
`CVE Numbering Authority (CNA) <https://www.cve.org/ProgramOrganization/CNAs>`_
covering all Apache projects and is the only group able to allocate IDs
to Apache Software Foundation project issues.


Not security vulnerabilities
============================

Apache NuttX RTOS is highly portable to over 15 different CPU architectures,
including microcontrollers with as tiny memory resources as single kilobytes
of RAM/Flash memory. Putting additional checks outside a generic nature would
dramatically impact final firmware performance and size.
**Function parameters and incoming data validation rests on the custom
application/firmware developer.**

Special care should be taken when handling:

 * syscalls.
 * pointers (always set to NULL before and after use).
 * structures (always initialize with ``{0}`` before use).
 * user controllable data (type and size).
 * network data.
 * dynamically allocated buffers.

.. note::
  If you find a generic problem in existing code base that
  may impact Confidentiality, Integrity, or Availability (i.e. information
  leak, denial of service, remote code execution) and is not your own custom
  application specific, please send us a security report.


Security Issues Handling
========================

Security related issues are handled in compliance with
`The Apache Security Team Guide <https://www.apache.org/security/>`_
and `Apache Committers Security Guide
<https://www.apache.org/security/committers.html>`_.
Please read these documents carefully before submitting and/or
handling a security vulnerabilities. Below is an extract of the information.

.. warning::
  Do not enter details of security vulnerabilities in a project's public
  bug tracker, issues, or pull requests. Do not make information about
  the vulnerability public until it is formally announced at the end
  of this process. Messages associated with any commits should not make
  any reference to the security nature of the commit.


1. Report:

  1. Please report potential security vulnerabilities over email to
     security@nuttx.apache.org **before disclosing them in any public form**.
     This enables responsible disclosure by providing a fix for everyone
     impacted before details are made public. Messages sent to our security@
     address are automatically copied to security@apache.org.

  2. Please send one plain-text, unencrypted, email for each vulnerability
     you are reporting. We may ask you to resubmit your report if you send
     it as an image, movie, HTML, or PDF attachment when you could as easily
     describe it with plain text.

  3. Do not enter details of security vulnerabilities in a project's public
     bug tracker, issues, or pull requests. **Do not make information about
     the vulnerability public until it is formally announced at the end
     of this process.** Messages associated with any commits should not make
     any reference to the security nature of the commit.

  4. Security fixes are usually part of the standard release cycle, but for
     urgent cases special patch releases may be created to address the issue.
     In order to keep this process smooth please provide us with as much
     details as possible. **Reproducible examples, proof-of-concept code,
     but most importantly fix patches are more than welcome.**

  5. There are problems that we are well aware of, and have been reported
     to us many times, but we do not classify as a security vulnerability, see
     `Not security vulnerabilities`_ for details.
     Please consider reporting them as Issue or Pull Request on GitHub instead.

  6. The project team sends an e-mail to the original reporter to acknowledge
     the report, with a copy to the relevant security mailing list.

2. Investigation:

  1. The project team investigates the report and either rejects or accepts it.

  2. Project team members may share information about the vulnerability
     with domain experts (including colleagues at their employer) at the
     discretion of the project's security team, providing that they make
     clear that the **information is not for public disclosure.**

  3. If the project team **rejects** the report, the team writes to the
     reporter to explain why, with a copy to the relevant security mailing
     list.

  4. If the project team **accepts** the report, the team writes to the
     reporter to let them know that they have accepted the report and that
     they are working on a fix or validating fix provided by the reporter.
     CVE ID is requested where problem details are reported upon resolution.

3. Resolution:

  1. The project team agrees on a fix on their private list.

  2. The project team requests a CVE (Common Vulnerabilities and Exposures)
     ID from the internal portal, https://cveprocess.apache.org.
     Apache Security Team can help determine if a report requires multiple
     CVE IDs or if multiple reports should be merged under a single CVE ID.
     CVE ID can be shared with the reporter.

  3. The project team documents the details of the vulnerability and the fix
     on the internal portal. The portal generates draft announcement texts.
     For an example of an announcement see Tomcat's announcement of
     CVE-2008-2370. The level of detail to include in the report is a matter
     of judgement. Generally, reports should contain enough information to
     enable people to assess the risk the vulnerability poses for their own
     system, and no more. **Announcements do not normally include steps
     to reproduce the vulnerability.**

  4. Optionally CVE can be set into the REVIEW state to request a review from
     the Apache Security team. Discussion is possible using the "comment"
     feature which also sends the comments to the private mailing list(s).

  5. The project team provides the reporter with a copy of the fix and the
     draft vulnerability announcement for comment.

  6. The project team agrees on the fix, the announcement, and the release
     schedule with the reporter. If the reporter is unresponsive in a
     reasonable timeframe this should not block the project team from moving
     to the next steps, particularly if an issue is of high severity/impact.

  7. The project team commits the fix **without making any reference that
     the commit relates to a security vulnerability.**

  8. The project team creates a release that includes the fix.

4. Public announcement:

  1. After (or at the same time as) the release announcement, the project
     team announces the vulnerability and the fix. CVE status is set to READY
     in the internal portal, that is also used to send emails.
     **This is the first point that any information regarding the
     vulnerability is made public.** The vulnerability announcement should
     be sent to the following destinations:

     a. the same destinations as the release announcement.
     b. the vulnerability reporter.
     c. the project's security list (or security@apache.org if
        the project does not have a dedicated security list).
     d. oss-security@lists.openwall.com (subscription not required).

  2. The project team updates the project's security pages.


NuttX CVEs
==========

CVE-2025-48769
--------------

* Title: fs/vfs/fs_rename: use after free.
* Published: 2026-01-01.
* Affected versions: >= 7.20 , < 12.11.0.
* Fixed in versions: 12.11.0.
* Type: `CWE-416 Use After Free <https://cwe.mitre.org/data/definitions/416.html>`_.
* Credits:

  * Finder: Liu, Richard Jiayang <rjliu3@illinois.edu>.
  * Remediation developer: Liu, Richard Jiayang <rjliu3@illinois.edu>.
  * Coordinator: Arnout Engelen <engelen@apache.org>.
  * Coordinator: Tomek CEDRO <cederom@apache.org>.
  * Remediation reviewer: Xiang Xiao <xiaoxiang@apache.org>.
  * Remediation reviewer: Jiuzhu Dong <jiuzhudong@apache.org>.

* References:

  * https://www.cve.org/CVERecord?id=CVE-2025-48769.
  * http://www.openwall.com/lists/oss-security/2025/12/31/11.
  * https://github.com/apache/nuttx/pull/16455.
  * https://lists.apache.org/thread/7m83v11ldfq7bvw72n9t5sccocczocjn.

Use After Free vulnerability was discovered in fs/vfs/fs_rename code of the
Apache NuttX RTOS, that due recursive implementation and single buffer use by
two different pointer variables allowed arbitrary user provided size buffer
reallocation and write to the previously freed heap chunk, that in specific
cases could cause unintended virtual filesystem rename/move operation results.
This issue affects Apache NuttX RTOS: from 7.20 before 12.11.0. Users of
virtual filesystem based services with write access especially when exposed
over the network (i.e. FTP) are affected and recommended to upgrade to
version 12.11.0 that fixes the issue.

CVE-2025-48768
--------------

* Title: fs/inode: fs_inoderemove root inode removal.
* Published: 2026-01-01.
* Affected versions: >= 10.0.0 , < 12.10.0.
* Fixed in version: 12.10.0.
* Type: `CWE-763 Release of Invalid Pointer or Reference <https://cwe.mitre.org/data/definitions/763.html>`_.
* Credits:

  * Finder: Liu, Richard Jiayang <rjliu3@illinois.edu>.
  * Remediation developer: Liu, Richard Jiayang <rjliu3@illinois.edu>.
  * Coordinator: Arnout Engelen <engelen@apache.org>.
  * Coordinator: Tomek CEDRO <cederom@apache.org>.
  * Remediation reviewer: Alan Carvalho de Assis <acassis@apache.org>.
  * Remediation reviewer: Tomek CEDRO <cederom@apache.org>.
  * Remediation reviewer: Xiang Xiao <xiaoxiang@apache.org>.
  * Remediation reviewer: Jiuzhu Dong <jiuzhudong@apache.org>.

* References:

  * https://www.cve.org/CVERecord?id=CVE-2025-48768.
  * http://www.openwall.com/lists/oss-security/2025/12/31/10.
  * https://github.com/apache/nuttx/pull/16437.
  * https://lists.apache.org/thread/nwo1kd08b7t3dyz082q2pghdxwvxwyvo.

Release of Invalid Pointer or Reference vulnerability was discovered in
fs/inode/fs_inoderemove code of the Apache NuttX RTOS that allowed root
filesystem inode removal leading to a debug assert trigger (that is disabled
by default), NULL pointer dereference (handled differently depending on the
target architecture), or in general, a Denial of Service. This issue affects
Apache NuttX RTOS: from 10.0.0 before 12.10.0. Users of filesystem based
services with write access that were exposed over the network (i.e. FTP)
are affected and recommended to upgrade to version 12.10.0 that fixes
the issue.

CVE-2025-47869
--------------

* Title: examples/xmlrpc: Fix calls buffers size.
* Published: 2025-06-16.
* Affected versions: >= 6.22 , < 12.9.0.
* Fixed in version: 12.9.0.
* Type: `CWE-119 Improper Restriction of Operations within the Bounds of a Memory Buffer <https://cwe.mitre.org/data/definitions/119.html>`_.
* Credits:

  * Reporter: Chánh Phạm <chanhphamviet@gmail.com>.
  * Remediation developer: Arnout Engelen <engelen@apache.org>.
  * Coordinator: Arnout Engelen <engelen@apache.org>.
  * Coordinator: Tomek CEDRO <cederom@apache.org>.
  * Remediation reviewer: Alan Carvalho de Assis <acassis@apache.org>.
  * Remediation reviewer: Alin Jerpelea <jerpelea@apache.org>.
  * Remediation reviewer: Lee, Lup Yuen <lupyuen@apache.org>.
  * Remediation reviewer: Xiang Xiao <xiaoxiang@apache.org>.
  * Remediation reviewer: Jianyu Wang <wangjianyu3@xiaomi.com>.

* References:

  * https://www.cve.org/CVERecord?id=CVE-2025-47869.
  * http://www.openwall.com/lists/oss-security/2025/06/14/2.
  * https://github.com/apache/nuttx-apps/pull/3027.
  * https://lists.apache.org/thread/306qcqyc3bpb2ozh015yxjo9kqs4jbvj.

Improper Restriction of Operations within the Bounds of a Memory Buffer
vulnerability was discovered in Apache NuttX RTOS apps/exapmles/xmlrpc
application. In this example application device stats structure that stored
remotely provided parameters had hardcoded buffer size which could lead to
buffer overflow. Structure members buffers were updated to valid size of
CONFIG_XMLRPC_STRINGSIZE+1. This issue affects Apache NuttX RTOS users that
may have used or base their code on example application as presented in
releases from 6.22 before 12.9.0. Users of XMLRPC in Apache NuttX RTOS are
advised to review their code for this pattern and update buffer sizes
as presented in the version of the example in release 12.9.0.

CVE-2025-47868
--------------

* Title: tools/bdf-converter.: tools/bdf-converter: Fix loop termination
  condition.
* Published: 2025-06-16.
* Affected versions: >= 6.9 , < 12.9.0.
* Fixed in version: 12.9.0.
* Type:

  * `CWE-787 Out-of-bounds Write <https://cwe.mitre.org/data/definitions/787.html>`_.
  * `CWE-122 Heap-based Buffer Overflow <https://cwe.mitre.org/data/definitions/122.html>`_.

* Credits:

  * Finder: Chánh Phạm <chanhphamviet@gmail.com>.
  * Remediation developer: Nathan Hartman <hartmannathan@apache.org>.
  * Coordinator: Arnout Engelen <engelen@apache.org>.
  * Coordinator: Tomek CEDRO <cederom@apache.org>
  * Remediation reviewer: Alan Carvalho de Assis <acassis@apache.org>.
  * Remediation reviewer: Alin Jerpelea <jerpelea@apache.com>.
  * Remediation reviewer: Lee, Lup Yuen <lupyuen@apache.org>.
  * Remediation reviewer: Nathan Hartman <hartmannathan@apache.org>.
  * Remediation reviewer: Simone Falsetti <simbit18@apache.org>.

* References:

  * https://www.cve.org/CVERecord?id=CVE-2025-47868.
  * http://www.openwall.com/lists/oss-security/2025/06/14/1.
  * https://github.com/apache/nuttx/pull/16000.
  * https://lists.apache.org/thread/p4o2lcqgspx3ws1n2p4wmoqbqow1w1pw.

Out-of-bounds Write resulting in possible Heap-based Buffer Overflow
vulnerability was discovered in tools/bdf-converter font conversion utility
that is part of Apache NuttX RTOS repository. This standalone program is
optional and neither part of NuttX RTOS nor Applications runtime, but active
bdf-converter users may be affected when this tool is exposed to external
provided user data data (i.e. publicly available automation). This issue
affects Apache NuttX: from 6.9 before 12.9.0. Users are recommended
to upgrade to version 12.9.0, which fixes the issue.

CVE-2025-35003
--------------

* Title: NuttX Bluetooth Stack HCI and UART DoS/RCE Vulnerabilities.
* Published: 2025-05-26.
* Affected versions: >= 7.25 , < 12.9.0.
* Fixed in version: 12.9.0.
* Type:

  * `CWE-119 Improper Restriction of Operations within the Bounds of a Memory Buffer <https://cwe.mitre.org/data/definitions/119.html>`_.
  * `CWE-121 Stack-based Buffer Overflow <https://cwe.mitre.org/data/definitions/121.html>`_.

* Credits:

  * Reporter: Chongqing Lei <leicq@seu.edu.cn>.
  * Reporter: Zhen Ling <zhenling@seu.edu.cn>.
  * Remediation developer: Chongqing Lei <leicq@seu.edu.cn>.
  * Coordinator: Arnout Engelen <engelen@apache.org>.
  * Coordinator: Tomek CEDRO <cederom@apache.org>.
  * Remediation reviewer: Lee, Lup Yuen <lupyuen@apache.org>.
  * Remediation reviewer: Xiang Xiao <xiaoxiang@apache.org>.

* References:

  * https://www.cve.org/CVERecord?id=CVE-2025-35003.
  * http://www.openwall.com/lists/oss-security/2025/05/26/1.
  * https://github.com/apache/nuttx/pull/16179.
  * https://lists.apache.org/thread/k4xzz3jhkx48zxw9vwmqrmm4hmg78vsj.

Improper Restriction of Operations within the Bounds of a Memory Buffer and
Stack-based Buffer Overflow vulnerabilities were discovered in Apache NuttX
RTOS Bluetooth Stack (HCI and UART components) that may result in system
crash, denial of service, or arbitrary code execution, after receiving
maliciously crafted packets. NuttX's Bluetooth HCI/UART stack users are
advised to upgrade to version 12.9.0, which fixes the identified
implementation issues.

CVE-2021-34125
--------------

* Published: 2023-03-09.
* Affected versions: PX4-Autopilot <= 1.11.3.
* Fixed in version:

  * nuttx#016873788280ca815ba886195535bbe601de6e48.
  * nuttx-apps#2fc1157f8585acc39f13a31612ebf890f41e76ca.
  * px4-autopilot#555f900cf52c0057e4c429ff3699c91911a21cab.

* References:

  * https://www.cve.org/CVERecord?id=CVE-2021-34125.
  * https://github.com/PX4/PX4-Autopilot/issues/17062.
  * https://github.com/PX4/PX4-Autopilot/pull/17264/commits/555f900cf52c0057e4c429ff3699c91911a21cab.
  * https://www.st.com/resource/en/application_note/dm00493651-introduction-to-stm32-microcontrollers-security-stmicroelectronics.pdf.
  * https://nuttx.apache.org/.
  * https://nuttx.apache.org/docs/latest/applications/nsh/commands.html#access-memory-mb-mh-and-mw.
  * https://gist.github.com/swkim101/f473b9a60e6d4635268402a2cd2025ac.
  * https://github.com/apache/incubator-nuttx/pull/3292/commits/016873788280ca815ba886195535bbe601de6e48.
  * https://github.com/apache/incubator-nuttx-apps/pull/647/commits/2fc1157f8585acc39f13a31612ebf890f41e76ca.

An issue discovered in Yuneec Mantis Q and PX4-Autopilot v1.11.3 and below
allow attacker to gain access to sensitive information via various nuttx
commands.

CVE-2021-26461
--------------

* Title:  malloc, realloc and memalign implementations are vulnerable
  to integer wrap-arounds.
* Published: 2021-06-21.
* Affected versions: < 10.1.0.
* Fixed in version: 10.1.0.
* Type: `CWE-190 Integer Overflow or Wraparound <https://cwe.mitre.org/data/definitions/190.html>`_.
* Credits: Apache NuttX would like to thank Omri Ben-Bassat of Section 52
  at Azure Defender for IoT of Microsoft Corp for bringing this issue
  to our attention.
* References:

  * https://www.cve.org/CVERecord?id=CVE-2021-26461.
  * https://lists.apache.org/thread.html/r806fccf8b003ae812d807c6c7d97950d44ed29b2713418cbe3f2bddd%40%3Cdev.nuttx.apache.org%3E.

Apache Nuttx Versions prior to 10.1.0 are vulnerable to integer wrap-around
in functions malloc, realloc and memalign. This improper memory assignment
can lead to arbitrary memory allocation, resulting in unexpected behavior
such as a crash or a remote code injection/execution.

CVE-2020-17529
--------------

* Title: Apache NuttX (incubating) Out of Bound Write from invalid
  fragmentation offset value specified in the IP header.
* Published: 2020-12-09.
* Affected versions: < 10.0.1.
* Fixed in version: 10.0.1.
* Type: `CWE-787 Out-of-bounds Write <https://cwe.mitre.org/data/definitions/787.html>`_.
* Credits: Apache NuttX would like to thank Forescout for reporting the issue.
* References:

  * https://www.cve.org/CVERecord?id=CVE-2020-17529.
  * http://www.openwall.com/lists/oss-security/2020/12/09/5.
  * https://lists.apache.org/thread.html/r4d71ae3ab96b589835b94ba7ac4cb88a704e7307bceefeab749366f3%40%3Cdev.nuttx.apache.org%3E.

Out-of-bounds Write vulnerability in TCP Stack of Apache NuttX (incubating)
versions up to and including 9.1.0 and 10.0.0 allows attacker to corrupt
memory by supplying and invalid fragmentation offset value specified
in the IP header. This is only impacts builds with both CONFIG_EXPERIMENTAL
and CONFIG_NET_TCP_REASSEMBLY build flags enabled.

CVE-2020-17528
--------------

* Title: Apache NuttX (incubating) Out of Bound Write from invalid TCP
  Urgent length.
* Published: 2020-12-09.
* Affected versions: < 10.0.1.
* Fixed in version: 10.0.1.
* Type: `CWE-787 Out-of-bounds Write <https://cwe.mitre.org/data/definitions/787.html>`_.
* Credits: Apache NuttX would like to thank Forescout for reporting the issue.
* References:

  * https://www.cve.org/CVERecord?id=CVE-2020-17528.
  * http://www.openwall.com/lists/oss-security/2020/12/09/4.
  * https://lists.apache.org/thread.html/r7f4215aba288660b41b7e731b6262c8275fa476e91e527a74d2888ea%40%3Cdev.nuttx.apache.org%3E.

Out-of-bounds Write vulnerability in TCP stack of Apache NuttX (incubating)
versions up to and including 9.1.0 and 10.0.0 allows attacker to corrupt
memory by supplying arbitrary urgent data pointer offsets within TCP packets
including beyond the length of the packet.

CVE-2020-1939
-------------

* Published: 2020-05-12.
* Affected versions: >= 6.15 , <= 8.2.
* Fixed in version: 9.0.0.
* References:

  * https://www.cve.org/CVERecord?id=CVE-2020-1939.
  * https://lists.apache.org/thread.html/re3adc65ff4d8d9c34e5bccba3941a28cbb0a47191c150df2727e101d%40%3Cdev.nuttx.apache.org%3E.

The Apache NuttX (Incubating) project provides an optional separate "apps"
repository which contains various optional components and example programs.
One of these, ftpd, had a NULL pointer dereference bug. The NuttX RTOS itself
is not affected. Users of the optional apps repository are affected only
if they have enabled ftpd. Versions 6.15 to 8.2 are affected.

CVE-2018-20578
--------------

* Published: 2018-12-28.
* Affected versions: < 7.27.
* Fixed in version: 7.27.
* References:

  * https://www.cve.org/CVERecord?id=CVE-2018-20578.
  * https://bitbucket.org/nuttx/nuttx/issues/119/denial-of-service-infinite-loop-while.
  * https://bitbucket.org/nuttx/nuttx/downloads/nuttx-7_27-README.txt.

An issue was discovered in NuttX before 7.27. The function
netlib_parsehttpurl() in apps/netutils/netlib/netlib_parsehttpurl.c
mishandles URLs longer than hostlen bytes (in the webclient, this is set
by default to 40), leading to an Infinite Loop.
The attack vector is the Location header of an HTTP 3xx response.

