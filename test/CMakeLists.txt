add_executable(jacobi jacobi.cpp ../delta/control/Jacobian.cpp ../delta/control/Kinematic.cpp)
target_link_libraries(jacobi eeros)
