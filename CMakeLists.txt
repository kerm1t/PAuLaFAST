cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# "../" --> output directory relative same to running in IDE
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)

#set (CMAKE_GENERATOR_PLATFORM "x64")
 
project(PAuLaFAST)

find_package(OpenGL REQUIRED)
#set(Boost_USE_STATIC_LIBS ON)
#find_package(PCL 1.9.1 REQUIRED COMPONENTS common io)
#find_package(PCL 1.8 REQUIRED)

#include_directories(./src ./imgui ./libs/glfw/include C:/winapp/PCL1.9.1/include/pcl-1.9 C:/winapp/PCL1.9.1/3rdParty/Boost/include/boost-1_68)
include_directories(./src ./imgui ./libs/glfw/include ${PCL_INCLUDE_DIRS})
link_directories(${CMAKE_SOURCE_DIR}/libs/glfw/debug/x64)# ${PCL_LIBRARY_DIRS})

#add_definitions(${PCL_DEFINITIONS})


add_executable (PAuLaFAST ./src/main.cpp ./src/imgui_impl_glfw.cpp ./imgui/imgui.cpp ./imgui/imgui.h ./imgui/imgui_draw.cpp ./src/paula.h ./src/paula_io.hpp #./src/paula_detect.hpp
./src/paula_visu.hpp ./src/visu_GL.hpp)

target_link_libraries (PAuLaFAST ${OPENGL_LIBRARIES} glfw3.lib Shlwapi.lib ${PCL_LIBRARIES})