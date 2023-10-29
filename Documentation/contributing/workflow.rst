.. todo:: update when workflow is settled

====================
Development Workflow
====================

NuttX development workflow is based around contributions submitted in the form
of GitHub Pull Requests (PR). This is true both for external contributors and
NuttX maintainers, as direct pushes to the repository are not allowed as a
general rule. Once submitted, your PR will be reviewed and checked using
Continuous Integration (CI) practices.

You should be aware of the following:

  - All contributions must adhere to the :doc:`Coding Standard <coding_style>`.
    You can check your files using ``nxstyle`` or complete patchsets using
    ``checkpatch`` script (both found in ``tools`` subdirectory of NuttX
    prepository). This check will also run automatically during CI to ensure
    conformance.

    Note that not all existing files in the repository are already adapted to
    conform to the standard as this is an ongoing effort. Thus, if you're
    submitting a patch to an existing file you may have to make the file conform
    to the standard, even if you are not responsible for those standard violations.

    It is also appreciated that you separate any styling fixes in a separate
    commit from the functional changes so that these are more easily readable
    during review.

  - Before starting work on any given non trivial contribution, do subscribe to
    the mailing list and ask about your idea to avoid wasted effort by going the
    wrong-route.

  - If you are submitting an original contribution (you wrote the code yourself
    from scratch) it will have to be submitted under the terms of the Apache 2.0
    License using the corresponding :ref:`header <contributing/coding_style:Appendix>`.

    Note that if you are working as an employee in a company, usually copyright
    belongs to the company and thus this means the company will have to authorize
    this and submit the appropriate license agreements.

  - If you are submitting third-party code:

    - Code from actively developed projects is not accepted to be included in
      NuttX (i.e.: creating a fork). It is expected that changes required in
      third-party code for NuttX support are to be implemented in these projects.
      As an intermediate solution, it is acceptable to include a patch to be applied
      to this third-party code, which will be pulled during built.

    - If this is from an inactive project, it may be considered for inclusion in
      NuttX, provided that licensing terms allow to do so and it is deemed of
      sufficient value to be included, considering that this code will have to
      be maintained in NuttX afterwards.

      Note that it is undesirable to included non Apache 2.0 Licensed code inside
      the repository, even if the license itself allows it (for example BSD License).

