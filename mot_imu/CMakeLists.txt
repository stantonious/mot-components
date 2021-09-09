
idf_component_register(SRCS "imu_task.c"
                       INCLUDE_DIRS . includes
                       REQUIRES mot_math core2forAWS buffers esp-dsp
                       )
