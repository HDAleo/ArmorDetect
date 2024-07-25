cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(Project)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 OpenCV 包
find_package(OpenCV REQUIRED)

# 查找线程库
find_package(Threads REQUIRED)

# 包含头文件目录
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

# 设置源文件
set(SOURCES
    main.cpp
    src/detector.cpp
    src/number_classifier.cpp
    src/positon.cpp
    src/kalman.cpp
    src/server.cpp
)

# 添加可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接库
target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS} 
    Threads::Threads
)
