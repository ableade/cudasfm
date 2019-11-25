cmake_minimum_required (VERSION 2.6)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(cudasfm)
include(ExternalProject)
set(LIB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/LIB)

set(CMAKE_CXX_FLAGS "${GCC_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${GCC_COMPILE_FLAGS}")

IF(MSVC)
  add_compile_options(/wd4514 /wd4267 /bigobj)
  add_definitions(-D_USE_MATH_DEFINES)
ELSE()
  IF (CMAKE_SYSTEM_PROCESSOR MATCHES "(arm64)|(ARM64)|(aarch64)|(AARCH64)")
    add_definitions (-march=armv8-a)
  ELSEIF (CMAKE_SYSTEM_PROCESSOR MATCHES 
          "(arm)|(ARM)|(armhf)|(ARMHF)|(armel)|(ARMEL)")
    add_definitions (-march=armv7-a)
  ELSE ()
    add_definitions (-march=native) #TODO use correct c++11 def once everybody has moved to gcc 4.7 # for now I even removed std=gnu++0x
  ENDIF()
  add_definitions (
    -O3
    -Wall
    -Wextra
    #-Werror
    -Wwrite-strings
    -Wno-unused-parameter
    -fno-strict-aliasing
  )
ENDIF()

find_package(Shomagick REQUIRED)
add_subdirectory(apps)