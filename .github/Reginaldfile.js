reginald.git.modifiedFiles.push(...reginald.git.addedFiles, ...reginald.git.removedFiles);

if (reginald.git.modifiedFiles.some(value => value.endsWith('.defs') || value.endsWith('Makefile'))) {
    reginald.warning("It seems that you have build-related changes, please ensure that this changes are also synchronized to CMake build");
 }