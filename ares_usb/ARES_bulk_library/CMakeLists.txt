# 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

# 定义项目名称
project(AresBulkExample LANGUAGES CXX)

# 设置 C++ 标准为 C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 libusb-1.0 库
# 在 macOS 上，通常需要通过 pkg-config 查找
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB_1 REQUIRED libusb-1.0)

# 查找 PThreads 库 (用于 std::thread)
find_package(Threads REQUIRED)

# 添加可执行文件目标
add_executable(ares_example
    example.cpp
    ares_protocol.cpp
)

# 添加 easy_example 可执行文件目标
add_executable(easy_example easy_example.cpp ares_protocol.cpp)

# 将 easy_example 链接到 libusb-1.0
# 如果你使用了 find_package:
# target_link_libraries(easy_example PRIVATE LibUSB::libusb-1.0)
# 或者直接指定库名 (更常见):
target_link_libraries(easy_example PRIVATE usb-1.0)

# 链接 libusb-1.0 库
# target_include_directories 用于添加头文件搜索路径
# target_link_libraries 用于链接库文件
target_include_directories(ares_example PRIVATE ${LIBUSB_1_INCLUDE_DIRS})
target_link_libraries(ares_example PRIVATE ${LIBUSB_1_LIBRARIES})

# 链接 PThreads 库
target_link_libraries(ares_example PRIVATE Threads::Threads)

# (可选) 如果头文件和源文件不在同一目录，添加头文件目录
# target_include_directories(ares_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
# 打印找到的 libusb 信息 (可选，用于调试)
message(STATUS "Found libusb-1.0 include dirs: ${LIBUSB_1_INCLUDE_DIRS}")
message(STATUS "Found libusb-1.0 libraries: ${LIBUSB_1_LIBRARIES}")