cmake_minimum_required(VERSION 3.8)
project(makne_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # nav_msgs 패키지 추가

# 메시지 및 서비스 파일 정의 및 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MsgTest.msg"
  "msg/Pose2D.msg"
  "srv/SetPointList.srv"
  "srv/GetStatus.srv"
  "srv/BoolSignal.srv"
  DEPENDENCIES geometry_msgs nav_msgs  # geometry_msgs 및 nav_msgs 의존성 명시
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
