#cmake version
cmake_minimum_required(VERSION 3.15)

project(MatchCrater VERSION 1.3.3 LANGUAGES CXX)

#set the c++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置Release模式下的编译优化选项
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

#set(GDAL_DIR "/home/guderian/anaconda3/envs/cpp/lib/cmake/gdal")
#set(OPENCV_DIR "/home/guderian/anaconda3/envs/cpp/lib/cmake/opencv4")
#set(CMAKE_PREFIX_PATH "/home/guderian/anaconda3/envs/cpp/lib/cmake/Qt5")

# 添加 GDAL 库
find_package(GDAL REQUIRED)
# 添加 OpenCV 库
find_package(OpenCV REQUIRED)
# 添加 Qt5 库
find_package(Qt5 COMPONENTS Widgets REQUIRED)
# 添加 Boost 库
find_package(Boost REQUIRED COMPONENTS program_options)
# 添加 JSON 库
find_package(nlohmann_json REQUIRED)


# 打印 GDAL 库的信息
message("GDAL_INCLUDE_DIRS: ${GDAL_INCLUDE_DIRS}")
message("GDAL_LIBRARIES: ${GDAL_LIBRARIES}")
# 打印 OpenCV 库的信息
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBS: ${OpenCV_LIBS}")
# 打印 Qt5 库的信息
message("Qt5Widgets_INCLUDE_DIRS: ${Qt5Widgets_INCLUDE_DIRS}")
message("Qt5Widgets_LIBRARIES: ${Qt5Widgets_LIBRARIES}")
# 打印 Boost 库的信息
message("Boost_VERSION: ${Boost_VERSION}")
message("Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message("Boost_LIBRARIES: ${Boost_LIBRARIES}")
message("Boost_PROGRAM_OPTIONS_FOUND: ${Boost_PROGRAM_OPTIONS_FOUND}")
message("Boost_PROGRAM_OPTIONS_LIBRARY: ${Boost_PROGRAM_OPTIONS_LIBRARY}")
# 打印 JSON 库的信息
message("nlohmann_json_VERSION: ${nlohmann_json_VERSION}")
message("nlohmann/json 库头文件路径:")
get_target_property(json_include_dirs nlohmann_json::nlohmann_json INTERFACE_INCLUDE_DIRECTORIES)
foreach(dir ${json_include_dirs})
    message(STATUS "  ${dir}")
endforeach()

message("nlohmann/json 库链接的库:")
get_target_property(json_linked_libs nlohmann_json::nlohmann_json INTERFACE_LINK_LIBRARIES)
if(json_linked_libs)
    foreach(lib ${json_linked_libs})
        message(STATUS "  ${lib}")
    endforeach()
else()
    message(STATUS "  无（nlohmann/json 是 header - only 库）")
endif()

# 包含自定义头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/IO)
include_directories(${MODULE_INCLUDE_DIRS})  

message("${CMAKE_SOURCE_DIR}/include")


#add the executable
add_executable(MatchCrater
${MODULE_SOURCES}
src/main.cpp
src/crater.cpp
src/GDALTransformer.cpp
src/debug.cxx
src/IO/getdata.cpp
src/IO/json_serializer.cpp
src/kdtree.cpp
src/MatchingCrater.cpp
)


#link the libraries
target_link_libraries(MatchCrater ${OpenCV_LIBS})
target_link_libraries(MatchCrater GDAL::GDAL)
target_link_libraries(MatchCrater Qt5::Widgets)
target_link_libraries(MatchCrater Boost::program_options)
target_link_libraries(MatchCrater nlohmann_json::nlohmann_json)
