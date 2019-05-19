

cmake_minimum_required(VERSION 3.6)
set( PROJ_NAME m2w_test )
project( ${PROJ_NAME} CXX )
set(CMAKE_CXX_STANDARD 14)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

set( TARGET_NAME          ${PROJ_NAME}          )
set( GTEST_DIR            ../sub/googletest  )

set( GTEST_FILES
  ${GTEST_DIR}/googletest/src/gtest-all.cc
  ${GTEST_DIR}/googlemock/src/gmock-all.cc
  )

add_definitions( -Dgtest_disable_pthreads=ON )

FIND_PACKAGE(Boost COMPONENTS system)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
ENDIF()


include_directories( SYSTEM ${GTEST_DIR}/googletest )
include_directories( SYSTEM ${GTEST_DIR}/googletest/include )
include_directories( SYSTEM ${GTEST_DIR}/googlemock )
include_directories( SYSTEM ${GTEST_DIR}/googlemock/include )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR})

# Test-executable
add_executable( ${TARGET_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${GTEST_FILES} )
target_link_libraries(${TARGET_NAME} pthread )
target_link_libraries(${TARGET_NAME} ${Boost_LIBRARIES})


