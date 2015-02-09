###########################################################
#           Main cmake file for the delta robot           #
###########################################################

set(EEDURO_ROOT ${PROJECT_SOURCE_DIR})

project(eeduro-delta)

macro(add_delta_sources)
  file(RELATIVE_PATH _relPath "${EEDURO_ROOT}/delta" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_src ${ARGN})
        if(_relPath)
            list(APPEND DELTA_SRCS "${_relPath}/${_src}")
        else()
            list(APPEND DELTA_SRCS "${_src}")
        endif()
    endforeach()
    if(_relPath)
        # propagate DELTA_SRCS to parent directory
        set(DELTA_SRCS ${DELTA_SRCS} PARENT_SCOPE)
    endif()
endmacro()

include_directories(${EEDURO_ROOT}/eeduro/include)

add_subdirectory(control)
add_subdirectory(safety)
add_subdirectory(sequencer)

# Main executable for delta robot
add_delta_sources(delta.cpp Calibration.cpp)
add_executable(eeduro-delta ${DELTA_SRCS})
target_link_libraries(eeduro-delta eeduro eeros)
install(TARGETS eeduro-delta RUNTIME DESTINATION bin)

# # Header files
