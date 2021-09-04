
idf_component_register(SRCS "mot-imu-model.cc" "mot-imu-tf.cpp"
                       INCLUDE_DIRS . includes
                       REQUIRES tfmicro mot_imu
                       )
