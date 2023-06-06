cmake_minimum_required(VERSION 3.15...3.26)
project(${SKBUILD_PROJECT_NAME} VERSION ${SKBUILD_PROJECT_VERSION})

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    endif()
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/master/conan.cmake" "${CMAKE_BINARY_DIR}/conan.cmake")
endif ()
include(${CMAKE_BINARY_DIR}/conan.cmake)

add_compile_definitions(PX_PHYSX_STATIC_LIB)
conan_add_remote(NAME conan-center URL https://center.conan.io)
conan_cmake_run(REQUIRES
        tinyobjloader/1.0.6
        eigen/3.3.7
        physx/4.1.1
        BASIC_SETUP BUILD missing)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

find_package(codecov)

include_directories(include)

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_pyphysx MODULE src/pyphysx.cpp)
target_compile_definitions(_pyphysx PRIVATE VERSION_INFO=${PROJECT_VERSION})
target_link_libraries(_pyphysx PUBLIC ${CONAN_LIBS})

install(TARGETS _pyphysx DESTINATION pyphysx)

add_coverage(_pyphysx)
coverage_evaluate()
