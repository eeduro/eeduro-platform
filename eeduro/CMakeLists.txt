###########################################################
#         Main cmake file for the EEDURO library          #
###########################################################

project(eeduro-lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(eeduro src/Board.cpp)
target_link_libraries(eeduro eeros)
install(TARGETS eeduro ARCHIVE DESTINATION lib LIBRARY DESTINATION lib COMPONENT library)
