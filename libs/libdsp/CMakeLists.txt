if (CONFIG_LIBDSP)
  nuttx_add_library(dsp
    lib_pid.c lib_svm.c lib_transform.c lib_observer.c lib_foc.c
    lib_misc.c lib_motor.c lib_pmsm_model.c lib_pid_b16.c lib_svm_b16.c
    lib_transform_b16.c lib_foc_b16.c lib_misc_b16.c lib_motor_b16.c
    lib_pmsm_model_b16.c)
endif()
