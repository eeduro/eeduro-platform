include_directories(${PROJECT_SOURCE_DIR}/eeduro/include)

add_executable(jacobi jacobi.cpp ../delta/control/Jacobian.cpp ../delta/control/Kinematic.cpp)
target_link_libraries(jacobi eeros)

add_executable(led led.cpp)
target_link_libraries(led eeduro)

add_executable(button button.cpp)
target_link_libraries(button eeduro)

add_executable(encoder encoder.cpp)
target_link_libraries(encoder eeduro)

add_executable(magnet magnet.cpp)
target_link_libraries(magnet eeduro)

add_executable(voltage voltage.cpp)
target_link_libraries(voltage eeduro eeros)
