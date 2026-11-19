cmake_minimum_required(VERSION 3.10)
project(franka_safe CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Franka REQUIRED)

foreach(prog safe_nudge safe_wrist safe_record safe_goto safe_sequence safe_cycle safe_gripper safe_fk gripper_force_probe safe_compliant safe_move_nostop safe_soft_move)
  add_executable(${prog} ${prog}.cpp)
  target_link_libraries(${prog} PRIVATE franka)
endforeach()
