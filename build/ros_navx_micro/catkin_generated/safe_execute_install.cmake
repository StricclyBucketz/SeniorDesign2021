execute_process(COMMAND "/home/thechild/catkin_ws/build/ros_navx_micro/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/thechild/catkin_ws/build/ros_navx_micro/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
