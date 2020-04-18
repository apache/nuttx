# Pull Request Template

## Title guidelines

Following the guidelines for writting good commit messages (https://chris.beams.io/posts/git-commit/) and creating a meaningful title is key in effective team communication.

**do's**
- stm32h7:  Add SDMMC Support
- nsh:  Separate `source` and `sh` for POSIX compliance
- nxstyle:  Fixed Camel case detection
- drivers/serial:  Fixed style violation

**dont's**
- fixed bug
- nxstyle
- PR #12

## Description

**Describe problem solved by the PR**
A clear and concise description of the problem, if any, this PR will solve. Please also include relevant motivation and context: E.g. The current nsh sh command violates the POSIX ...

List any dependencies that are required for this change.

**Describe your solution**
A clear and concise description of what you have implemented.

**Describe possible alternatives**
A clear and concise description of alternative solutions  you've considered.

**Additional context**
Add any other context or screenshots for the pr.

Fixes # (issue)

## Type of change

Delete options that are not relevant.

- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] This change requires a documentation update

## How Has This Been Tested?

Please describe the tests that you ran to verify your changes. Provide instructions so we can reproduce. Please also list any relevant details for your test configuration

- [ ] Test A
- [ ] Test B

**Test Configuration**:

* Nuttx board/config:
* Hardware:
* Toolchain:

## Checklist:

- [ ] My code follows the style guidelines of this project (NEED link to how to run checkpatch)
- [ ] I have performed a self-review of my own code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to the documentation
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix is effective or that my feature works
- [ ] New and existing unit tests pass locally with my changes
- [ ] Any dependent changes have been merged and published in downstream modules
- [ ] I have checked my code and corrected any misspellings (NEED link to how to run checkpatch spelling)
