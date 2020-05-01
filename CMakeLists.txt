cmake_minimum_required(VERSION 2.8.12)
project(pyphysx)

set(CMAKE_CXX_STANDARD 14)

if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/master/conan.cmake" "${CMAKE_BINARY_DIR}/conan.cmake")
endif ()
include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_run(REQUIRES
        tinyobjloader/1.0.6
        pybind11/2.3.0@conan/stable
        eigen/3.3.7@conan/stable
        physx/4.1.1
        BASIC_SETUP BUILD missing)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()
find_package(pybind11 REQUIRED)

include_directories(include)
pybind11_add_module(pyphysx MODULE NO_EXTRAS src/pyphysx.cpp)
target_link_libraries(pyphysx PUBLIC ${CONAN_LIBS})
