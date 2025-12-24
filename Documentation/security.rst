========
Security
========

.. toctree::

Known vulnerabilities
=====================

Apache NuttX RTOS vulnerabilities are labelled with CVE (Common
Vulnerabilities and Exposures) identifiers. List of known, responsibly
disclosed, and fixed vulnerabilities are publicly available online at
`CVE.ORG <https://www.cve.org/CVERecord/SearchResults?query=nuttx>`_.
Offline bundled version is located at the bottom of this page in the
`NuttX CVEs`_ section.

Reporting Vulnerabilities
=========================

Security related issues are handled in compliance with
`The Apache Security Team Guide <https://www.apache.org/security/>`_
and `Apache Committers Security Guide
<https://www.apache.org/security/committers.html>`_.
Please read these documents carefully before submitting and/or
handling a security vulnerability.

.. warning::
  Do not enter details of security vulnerabilities in a project's public
  bug tracker, issues, or pull requests. Do not make information about
  the vulnerability public until it is formally announced at the end
  of this process. Messages associated with any commits should not make
  any reference to the security nature of the commit.


Below is an extract of the most important information:

1. Please report potential security vulnerabilities over email to
   security@apache.org and security@nuttx.apache.org **before disclosing
   them in any public form**. This enables responsible disclosure by
   providing a fix for everyone impacted before details are made public.

2. Please send one plain-text, unencrypted, email for each vulnerability
   you are reporting. We may ask you to resubmit your report if you send
   it as an image, movie, HTML, or PDF attachment when you could as easily
   describe it with plain text.

3. Do not enter details of security vulnerabilities in a project's public
   bug tracker, issues, or pull requests. Do not make information about
   the vulnerability public until it is formally announced at the end
   of this process. Messages associated with any commits should not make
   any reference to the security nature of the commit.

4. Security fixes are usually part of the standard release cycle, but for
   urgent cases special patch releases may be created to address the issue.
   In order to keep this process smooth please provide us with as much
   details as possible. **Reproducible examples, proof-of-concept code,
   but most importanly fix patches are more than welcome.**

5. There are problems that we are well aware of, and have been reported
   to us many times, but we do not class as a security vulnerability, see
   `Not security vulnerabilities`_ for details.
   Please consider reporting them as Issue or Pull Request on GitHub instead.

6. The project team sends an e-mail to the original reporter to acknowledge
   the report, with a copy to the relevant security mailing list.

7. The project team investigates the report and either rejects or accepts it.
   Project team members may share information about the vulnerability with
   domain experts (including colleagues at their employer) at the discretion
   of the project's security team, providing that they make clear that
   the information is not for public disclosure.

8. If the project team rejects the report, the team writes to the reporter
   to explain why, with a copy to the relevant security mailing list.

9. If the project team accepts the report, the team writes to the reporter
   to let them know that they have accepted the report and that they are
   working on a fix.

10. The project team agrees on a fix on their private list.

11. The project team requests a CVE (Common Vulnerabilities and Exposures)
    ID from the internal portal, https://cveprocess.apache.org.
    Apache Security Team can help determine if a report requires multiple
    CVE IDs or if multiple reports should be merged under a single CVE ID.
    CVE ID can be shared with the reporter.

12. The project team documents the details of the vulnerability and the fix
    on the internal portal. The portal generates draft announcement texts.
    For an example of an announcement see Tomcat's announcement of
    CVE-2008-2370. The level of detail to include in the report is a matter
    of judgement. Generally, reports should contain enough information to
    enable people to assess the risk the vulnerability poses for their own
    system, and no more. **Announcements do not normally include steps
    to reproduce the vulnerability.**

13. Optionally CVE can be set into the REVIEW state to request a review from
    the Apache Security team. Discussion is possible using the 'comment'
    feature which also sends the comments to the private mailing list(s).

14. The project team provides the reporter with a copy of the fix and the
    draft vulnerability announcement for comment.

15. The project team agrees on the fix, the announcement, and the release
    schedule with the reporter. If the reporter is unresponsive in a
    reasonable timeframe this should not block the project team from moving
    to the next steps, particularly if an issue is of high severity/impact.

16. The project team commits the fix **without making any reference that
    the commit relates to a security vulnerability.**

17. The project team creates a release that includes the fix.

18. After (or at the same time as) the release announcement, the project
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

19. The project team updates the project's security pages.


Not security vulnerabilities
============================

Apache NuttX RTOS is highly portable to over 15 different CPU architectures,
including microcontrollers with as tiny memory resources as single kilobytes
of RAM/Flash memory. Putting additional checks outside a generic nature would
dramatically impact final firmware performance and size.
**Function parameters and incoming data validation rests on the 
custom application/firmware developer.**

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

NuttX CVEs
==========

TODO
