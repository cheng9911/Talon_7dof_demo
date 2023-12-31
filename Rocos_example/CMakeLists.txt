cmake_minimum_required(VERSION 3.10)

project(rocos_test)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
list(APPEND CMAKE_PREFIX_PATH /opt/rocos/app)
list(APPEND CMAKE_PREFIX_PATH /opt/grpc)
include_directories(/opt/grpc/include)

find_package(rocos_app REQUIRED)
find_package(Eigen3 REQUIRED)
add_subdirectory(3rdparty/gflags)

set( List_Cpp
demo 
demo_cal
demo_torque  )
set(LIST ${List_Cpp} )
foreach(list ${LIST})

  add_executable(${list} src/${list}.cpp  )
  target_link_libraries(${list} PUBLIC rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )

endforeach()

# add_executable(demo src/demo.cpp)
# add_executable(demo_cal src/demo_cal.cpp)
# add_executable(demo_torque src/demo_torque.cpp)
# target_link_libraries(demo PUBLIC rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )

# target_link_libraries(demo_cal PUBLIC rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )

# target_link_libraries(demo_torque PUBLIC rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )


file(GLOB_RECURSE CONFIG_FILE config/*.urdf config/public.key config/license)
file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin/config)
