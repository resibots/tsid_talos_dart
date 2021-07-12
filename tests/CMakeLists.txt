find_package(Boost COMPONENTS unit_test_framework)

set(BINARY ${CMAKE_PROJECT_NAME}_test)

# the tests require Dart to run
if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
        find_package(RobotDART REQUIRED COMPONENTS Simu OPTIONAL_COMPONENTS Magnum)
        set(TEST_SOURCES test_determinism.cpp test_behaviors_talos.cpp )
endif()

foreach(TEST ${TEST_SOURCES})
        string(REPLACE ".cpp" "" TEST_NAME ${TEST})
        message("TEST: ${TEST_NAME}")
        add_executable(${TEST_NAME} ${TEST})
        add_test(NAME ${TEST_NAME}  COMMAND ${TEST_NAME})
        target_link_libraries(${TEST_NAME} PUBLIC inria_wbc RobotDART::Simu Boost::unit_test_framework)
        # having the graphics can be helpful for debugging (to see what happens)
        if(RobotDART_Magnum_FOUND)
                add_executable(${TEST_NAME}_graphics ${TEST})
                target_link_libraries(${TEST_NAME}_graphics PUBLIC inria_wbc RobotDART::Magnum RobotDART::Simu Boost::unit_test_framework)
        endif()
endforeach()


