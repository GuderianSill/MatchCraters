#cmake version
cmake_minimum_required(VERSION 3.15)

project(MatchCrater VERSION 1.0 LANGUAGES CXX)

#set the c++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置Release模式下的编译优化选项
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(GDAL_DIR "/home/guderian/anaconda3/envs/cpp/lib/cmake/gdal")
set(OPENCV_DIR "/home/guderian/anaconda3/envs/cpp/lib/cmake/opencv4")

# 添加 GDAL 库
find_package(GDAL REQUIRED)
# 添加 OpenCV 库
find_package(OpenCV REQUIRED)

# 打印 GDAL 库的信息
message("GDAL_INCLUDE_DIRS: ${GDAL_INCLUDE_DIRS}")
message("GDAL_LIBRARIES: ${GDAL_LIBRARIES}")
# 打印 OpenCV 库的信息
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBS: ${OpenCV_LIBS}")

# 包含自定义头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)

message("${CMAKE_SOURCE_DIR}/include")

#add the executable
add_executable(MatchCrater 
src/main.cpp
src/crater.cpp
src/debug.cxx
src/getdata.cpp
src/kdtree.cpp
src/MatchingCrater.cpp
)


#link the libraries
target_link_libraries(MatchCrater ${OpenCV_LIBS})
target_link_libraries(MatchCrater GDAL::GDAL)