cmake_minimum_required(VERSION 3.13)
project(inmarsatc_demodulator)

set(CMAKE_INSTALL_PREFIX "/usr/")
include(GNUInstallDirs)

if (MSVC)
    set(CMAKE_CXX_FLAGS "-O2 /std:c++17 /EHsc")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-O3 -std=c++17 -Wno-unused-command-line-argument -undefined dynamic_lookup")
else ()
    set(CMAKE_CXX_FLAGS "-O3 -std=c++17")
endif ()

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h" "src/*.hpp"  "src/*.c")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")

add_library(inmarsatc_demodulator SHARED ${SRC})
target_link_libraries(inmarsatc_demodulator PRIVATE sdrpp_core inmarsatc_decoder)
target_include_directories(inmarsatc_demodulator PRIVATE "src/" "/usr/include/sdrpp_core/" "/usr/include/sdrpp_core/imgui")
set_target_properties(inmarsatc_demodulator PROPERTIES PREFIX "")

# Install directives
install(TARGETS inmarsatc_demodulator DESTINATION ${CMAKE_INSTALL_LIBDIR}/sdrpp/plugins)
