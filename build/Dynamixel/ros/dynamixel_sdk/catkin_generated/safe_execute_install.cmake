execute_process(COMMAND "/mnt/c/Users/david/Desktop/HexaPod/build/Dynamixel/ros/dynamixel_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/mnt/c/Users/david/Desktop/HexaPod/build/Dynamixel/ros/dynamixel_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
