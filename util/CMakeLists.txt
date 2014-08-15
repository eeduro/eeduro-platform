
include_directories(${CMAKE_SOURCE_DIR}/eeduro/include)

add_executable(eeduro-ctrl ctrl.cpp)
target_link_libraries(eeduro-ctrl eeduro)
INSTALL(TARGETS eeduro-ctrl RUNTIME DESTINATION bin)
