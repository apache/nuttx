# nuttx_add_romfs
#  Generates a ROMFS image in a C array, which is built to an OBJECT
#  library.
#
# Parameters:
# - NAME: determines the romfs label and name of target (romfs_${NAME})
# - PATH: the directory that will be used to create the ROMFS

function(nuttx_add_romfs)
  nuttx_parse_function_args(
    FUNC nuttx_add_romfs
    ONE_VALUE NAME MOUNTPOINT PATH
    OPTIONS REQUIRED NAME PATH ARGN ${ARGN}
  )

  add_custom_command(
    OUTPUT romfs_${NAME}.c
    COMMAND ${CMAKE_COMMAND} -E make_directory romfs_${NAME}
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PATH} romfs_${NAME}
    COMMAND genromfs -f romfs.img -d romfs_${NAME} -V ${NAME}
    COMMAND xxd -i romfs.img > romfs_${NAME}.c
    COMMAND sed -ri 's/^unsigned/const unsigned/g' romfs_${NAME}.c
    COMMAND ${CMAKE_COMMAND} -E remove romfs.img
  )

  add_library(romfs_${NAME} OBJECT romfs_${NAME}.c)
  add_dependencies(romfs_${NAME} nuttx_context)
endfunction()
