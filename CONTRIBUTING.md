# Contributing to Apache NuttX RTOS

Hi! Thank you for your interest in contributing to Apache NuttX RTOS :-)

## Guidelines

To help us successfully review your contribution, it is very
important that you follow these guidelines.

If you need more information please read the [Full Contributing Documentation](https://nuttx.apache.org/docs/latest/contributing/index.html).

### Coding Standard

* You should follow [NuttX C Coding Standard](https://nuttx.apache.org/docs/latest/contributing/coding_style.html).

* Your code will be automatically checked by GitHub Continuous Integration
  (CI) system. If you see the "check" step fails, it is possible that this
  happens due to style errors.

* Note that we require you to solve these issues and adapt all modified files
  even if you didn't introduce the problem yourself (this way,
  every contribution gets us closer to compliance).


### Commits

* Each commit **must** contain a meaningful **commit message** that consist of:
  * **topic** (mandatory).
  * **description** (optional, separate with blank line from topic).

* **Prefix** commit topic with a functional context
  (i.e.  `sched: Fixed compiler warning.`).

* Provide only **signed commits** (`git commit -s`) with valid author
  and email fields.

* Valid commit message example:

    ```
    net/can: Add g_ prefix to can_dlc_to_len and len_to_can_dlc.

    Add g_ prefix to can_dlc_to_len and len_to_can_dlc to
    follow NuttX coding style conventions for global symbols,
    improving code readability and maintainability.
    * you can also use bullet points.
    * to note different thing briefly.

    Signed-off-by: AuthorName <Valid@EmailAddress>
    ```

* If you create a proper commit message as explained above, the first line
  will be automatically used as pull-request title and the rest is added as
  description. Use it as a starting point to describe your Pull-Request (PR).


### Pull Requests

* Be sure to **fill in the pull-request report** with meaningful content.
  Be very descriptive, take your time. Good explanation, as for someone who
  can see the issue for the first time, will help understand the change and
  improve the assessment process. Please provide short descriptions even for
  a trivial change.

  * **Summary:** It is important to provide information on why change is
    necessary, what it exactly does and how, if new feature shows up,
    provide references (dependencies, similar problems and solutions), etc.

  * **Impact:** State how (where applicable) that change affects users, build
    process, hardware, documentation, security, compatibility, etc.

  * **Testing:** Please provide details on how did you verify the change,
    what Host was used for build (OS, CPU, compiler, ..), what Target was
    used for verification (arch, board:config, ..), etc.
    Providing build and runtime logs from before and after change is highly
    appreciated.

* Introduce only one functional change per pull-request.

* Provide finished and verified solutions.
  Clearly mark Work-In-Progress pull requests not yet ready for assessment.


## References

### Example Pull Request Report

#### Summary

  * Why change is necessary (fix, update, new feature)?
  * What functional part of the code is being changed?
  * How does the change exactly work (what will change and how)?
  * Related [NuttX Issue](https://github.com/apache/nuttx/issues) reference if applicable.
  * Related NuttX Apps [Issue](https://github.com/apache/nuttx-apps/issues) / [Pull Request](https://github.com/apache/nuttx-apps/pulls) reference if applicable.

#### Impact

  * Is new feature added? Is existing feature changed?
  * Impact on user (will user need to adapt to change)? NO / YES (please describe if yes).
  * Impact on build (will build process change)? NO / YES (please descibe if yes).
  * Impact on hardware (will arch(s) / board(s) / driver(s) change)? NO / YES (please describe if yes).
  * Impact on documentation (is update required / provided)? NO / YES (please describe if yes).
  * Impact on security (any sort of implications)? NO / YES (please describe if yes).
  * Impact on compatibility (backward/forward/interoperability)? NO / YES (please describe if yes).
  * Anything else to consider?

#### Testing

  I confirm that changes are verified on local setup and works as intended:
  * Build Host(s): OS (Linux,BSD,macOS,Windows,..), CPU(Intel,AMD,ARM), compiler(GCC,CLANG,version), etc.
  * Target(s): arch(sim,RISC-V,ARM,..), board:config, etc.

  Testing logs before change:

  ```
  runtime / build logs before change goes here
  ```

  Testing logs after change:

  ```
  runtime / build logs after change goes here
  ```

  How to repeat:
    * you can also provide steps on how to reproduce problem and verify the change.

#### PR verification

  * [ ] This PR introduces only one functional change.
  * [ ] I have updated all required description fields above.
  * [ ] My PR adheres to Contributing [Guidelines](https://github.com/apache/nuttx/blob/master/CONTRIBUTING.md) and [Documentation](https://nuttx.apache.org/docs/latest/contributing/index.html) (git commit title and message, coding standard, etc).
  * [ ] My PR is still work in progress (not ready for review).
  * [ ] My PR is ready for review and can be safely merged into a codebase.
