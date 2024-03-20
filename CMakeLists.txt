cmake_minimum_required( VERSION 3.3 )
project( generate_path )
# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )
# 打印 OpenCV_INCLUDE_DIRS 的目录
message(opencv_include_dir ${OpenCV_INCLUDE_DIRS})

# .h文件包含的目录
include_directories( ${PROJECT_SOURCE_DIR}/include)

add_executable(generate_path src/main.cpp src/generate_path.cpp)

link_directories( ${OpenCV_LIBRARY_DIRS} )
# 使编译好的可执行文件可以链接OpenCV库
target_link_libraries(generate_path ${OpenCV_LIBS} )


