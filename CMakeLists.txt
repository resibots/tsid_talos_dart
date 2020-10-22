cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(inria_wbc)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INRIA_WBC_VERSION 1.0.0)

option(COMPILE_ROBOT_DART_EXAMPLE "compile robot-dart example" ON)
option(COMPILE_ROBOT_DART_EXAMPLE_GRAPHICS "compile robot-dart example with graphics" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -std=c++14")

find_package(tsid REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

list(APPEND DEPENDENCIES_LIBS
                             pinocchio::pinocchio
                             tsid::tsid
                             ${YAML_CPP_LIBRARIES}
)

list(APPEND DEPENDENCIES_INCS
                             ${CMAKE_SOURCE_DIR}/include
)

set(INRIA_WBC_HEADERS
                            include/inria_wbc/controllers/talos_base_controller.hpp
                            include/inria_wbc/controllers/talos_pos_tracking.hpp
                            include/inria_wbc/behaviors/factory.hpp
                            include/inria_wbc/behaviors/talos_squat.hpp
                            include/inria_wbc/behaviors/talos_move_arm.hpp
                            include/inria_wbc/behaviors/talos_walk_on_spot.hpp
                            include/inria_wbc/utils/trajectory_handler.hpp
                            include/inria_wbc/utils/utils.hpp
                            include/inria_wbc/estimators/cop.hpp
                            include/inria_wbc/estimators/butterworth_filter.hpp


)

set(INRIA_WBC_SOURCES
                            src/controllers/talos_base_controller.cpp
                            src/controllers/talos_pos_tracking.cpp
                            src/behaviors/talos_squat.cpp
                            src/behaviors/talos_move_arm.cpp
                            src/behaviors/talos_walk_on_spot.cpp
                            src/estimators/cop.cpp
                            src/estimators/butterworth_filter.cpp


)

set(EXAMPLE_LIST 
                src/robot_dart/evaluate_cop.cpp
                src/robot_dart/test_controller.cpp
                src/robot_dart/test_masses.cpp
)

if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
    list(APPEND DEPENDENCIES_LIBS RobotDART::Simu)
endif()

add_library(inria_wbc SHARED ${INRIA_WBC_HEADERS} ${INRIA_WBC_SOURCES})

target_link_libraries(inria_wbc PUBLIC ${DEPENDENCIES_LIBS})

target_include_directories(inria_wbc
                                           PUBLIC
                                           $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/build/include>
                                           $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                                           $<INSTALL_INTERFACE:include>
                                           $<BUILD_INTERFACE:${DEPENDENCIES_INCS}>
)


function(compile_examples)
    foreach(EXAMPLE ${ARGN})
        string(REPLACE ".cpp" "" EXAMPLE_NAME ${EXAMPLE} )
        string(REPLACE "src/robot_dart/" "" EXAMPLE_NAME ${EXAMPLE_NAME} )
        message("COMPILING EXAMPLE ${EXAMPLE_NAME}")
        add_executable(${EXAMPLE_NAME} ${EXAMPLE})
        target_link_libraries(${EXAMPLE_NAME} PUBLIC inria_wbc Boost::program_options)
        if(RobotDART_Magnum_FOUND)
            add_executable(${EXAMPLE_NAME}_graphics ${EXAMPLE})
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC inria_wbc RobotDART::Magnum Boost::program_options)
        endif()
    endforeach()
endfunction()


if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
    # we ask for Magnum because we want to build the graphics
    if(${COMPILE_ROBOT_DART_EXAMPLE_GRAPHICS} STREQUAL "ON")
        find_package(RobotDART REQUIRED OPTIONAL_COMPONENTS Magnum)
    else()
        find_package(RobotDART REQUIRED)
    endif()
    # boost is only used in the test_controller binary
    find_package(Boost REQUIRED COMPONENTS program_options)
    compile_examples(${EXAMPLE_LIST})
endif()



generate_export_header(inria_wbc)

install(TARGETS inria_wbc
                               EXPORT targets
                               LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                               ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                               RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                               INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(DIRECTORY include/inria_wbc DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(LIBRARY inria_wbc)

configure_package_config_file("cmake/inria_wbc.cmake.in"
                              "${CMAKE_CURRENT_BINARY_DIR}/inria_wbcConfig.cmake"
                              INSTALL_DESTINATION 
                              "${LIBRARY_INSTALL_DIR}/cmake/inria_wbc"
                              PATH_VARS 
                              INCLUDE_INSTALL_DIR 
                              LIBRARY_INSTALL_DIR LIBRARY
                              DEPENDENCIES_LIBS DEPENDENCIES_INCS
)

install(EXPORT targets
                     FILE
                     inria_wbcTargets.cmake
                     DESTINATION
                     ${LIBRARY_INSTALL_DIR}/cmake/inria_wbc
)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/inria_wbcConfigVersion.cmake"
                                 VERSION ${INRIA_WBC_VERSION}
                                 COMPATIBILITY AnyNewerVersion
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/inria_wbcConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/inria_wbcConfigVersion.cmake"
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/lib/cmake/inria_wbc
)


install(DIRECTORY etc/  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/inria_wbc/etc)

ADD_SUBDIRECTORY(tests)


