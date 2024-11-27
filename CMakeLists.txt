cmake_minimum_required (VERSION 3.16)
project ("project")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

file(GLOB SRCS "./src/*.cpp" "./src/*.c" "./imgui/*.cpp" "./imgui/backends/*.cpp")
add_executable(project ${SRCS})
target_include_directories(project PRIVATE "./include" "./imgui" "./imgui/backends")

find_package(glfw3 REQUIRED PATHS /usr/lib/x86_64-linux-gnu/)
target_link_libraries(project dl GL glfw)

