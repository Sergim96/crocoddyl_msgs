  # -----------------------
  # C++ Tests (ROS1)
  # -----------------------
  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs whole_body_state_msgs)
  find_package(rostest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Development)
  find_package(pybind11_catkin REQUIRED)

  # add_compile_options(-std=c++17)

  catkin_add_gtest(test_solver_queue test_solver_queue.cpp)
  target_include_directories(
    test_solver_queue
    PRIVATE ${catkin_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/../include
            ${EIGEN3_INCLUDE_DIR}
            ${PINOCCHIO_INCLUDE_DIRS}
            ${pybind11_catkin_INCLUDE_DIRS}
            ${Python3_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS})
  target_link_libraries(
    test_solver_queue ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES}
    ${pybind11_catkin_LIBRARIES} ${Python3_LIBRARIES} ${Boost_PYTHON_LIBRARY})