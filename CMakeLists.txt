find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JpegImage.msg"
  DEPENDENCIES std_msgs
)

ament_package()