
set(EXAMPLE_LIST
	talos.cpp
    franka.cpp
    tiago.cpp
)


function(compile_examples)
    foreach(EXAMPLE ${ARGN})
        string(REPLACE ".cpp" "" EXAMPLE_NAME ${EXAMPLE} )
        string(REPLACE "src/robot_dart/" "" EXAMPLE_NAME ${EXAMPLE_NAME} )
        message("COMPILING EXAMPLE ${EXAMPLE_NAME}")
        add_executable(${EXAMPLE_NAME} ${EXAMPLE})
        set_target_properties(${EXAMPLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
        target_compile_definitions(${EXAMPLE_NAME} PUBLIC BOOST_MATH_DISABLE_FLOAT128=1)
        target_compile_features(${EXAMPLE_NAME} PUBLIC cxx_std_14)
        target_link_libraries(${EXAMPLE_NAME} PUBLIC inria_wbc Boost::program_options RobotDART::Simu)
        if(RobotDART_Magnum_FOUND)
            message("COMPILING EXAMPLE ${EXAMPLE_NAME}_graphics")
            add_executable(${EXAMPLE_NAME}_graphics ${EXAMPLE})
            set_target_properties(${EXAMPLE_NAME}_graphics PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
            target_compile_definitions(${EXAMPLE_NAME}_graphics PUBLIC BOOST_MATH_DISABLE_FLOAT128=1)
            target_compile_features(${EXAMPLE_NAME}_graphics PUBLIC cxx_std_14)
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC inria_wbc RobotDART::Magnum RobotDART::Simu Boost::program_options)
        endif()
    endforeach()
endfunction()


if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
    # we ask for Magnum because we want to build the graphics
    if(${COMPILE_ROBOT_DART_EXAMPLE_GRAPHICS} STREQUAL "ON")
        find_package(RobotDART REQUIRED COMPONENTS Simu OPTIONAL_COMPONENTS Magnum)
    else()
        find_package(RobotDART REQUIRED)
    endif()
    # boost is only used in the test_controller binary
    find_package(Boost REQUIRED COMPONENTS program_options)
    compile_examples(${EXAMPLE_LIST})
endif()




