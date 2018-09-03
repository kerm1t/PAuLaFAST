cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# "../" --> output directory relative same to running in IDE
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)

#set (CMAKE_GENERATOR_PLATFORM "x64")
 
project(PAuLaFAST)

find_package(PCL 1.8 REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ./src ./imgui ./libs/glfw/include)# ./libs/glew/include)
link_directories(${PCL_LIBRARY_DIRS} ../libs/glfw/lib-vc2015)# ../libs/glew/lib-x64-debug)
add_definitions(${PCL_DEFINITIONS})

add_executable (PAuLaFAST ./src/main.cpp ./src/imgui_impl_glfw.cpp ./imgui/imgui.cpp ./imgui/imgui.h ./imgui/imgui_draw.cpp ./src/paula.h ./src/paula_io.hpp ./src/paula_detect.hpp ./src/paula_visu.hpp ./src/visu_GL.hpp)
target_link_libraries (PAuLaFAST ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} glfw3.lib)# glew32d.lib)