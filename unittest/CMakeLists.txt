if(CATKIN_ENABLE_TESTING)
  # -----------------------
  # C++ Tests (ROS1)
  # -----------------------
  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs whole_body_state_msgs)
  find_package(rostest REQUIRED)

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

  # -----------------------
  # Python Tests (ROS1)
  # -----------------------
  set(${PROJECT_NAME}_PYTHON_TESTS solver_statistics solver_trajectory)

  foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
    add_rostest(crocoddyl_msgs.test ARGS test_name:=${TEST})
  endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

else() # ROS2
  # -----------------------
  # C++ Tests (ROS2)
  # -----------------------
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Boost REQUIRED COMPONENTS python314)
  find_package(Python3 REQUIRED COMPONENTS Development NumPy)
  find_package(realtime_tools REQUIRED)

  ament_add_gtest(test_solver_queue_ros2 test_solver_queue_ros2.cpp)
  if(TARGET test_solver_queue_ros2)
    rosidl_get_typesupport_target(test_cpp_typesupport_target ${PROJECT_NAME}
                                  rosidl_typesupport_cpp)
    target_include_directories(test_solver_queue_ros2
                               PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(
      test_solver_queue_ros2
      ${test_cpp_typesupport_target}
      Boost::python314
      Eigen3::Eigen
      Python3::Python
      Python3::NumPy
      pinocchio::pinocchio
      rclcpp::rclcpp
      realtime_tools::realtime_tools)
    set_tests_properties(test_solver_queue_ros2 PROPERTIES TIMEOUT 20)
  endif()

  # -----------------------
  # Python Tests (ROS2)
  # -----------------------
  set(${PROJECT_NAME}_PYTHON_TESTS
      solver_statistics solver_trajectory model whole_body_state
      whole_body_trajectory inertial_parameters)

  foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
    add_test(NAME "test_${TEST}"
             COMMAND ${PYTHON_EXECUTABLE}
                     "${PROJECT_SOURCE_DIR}/unittest/test_${TEST}.py")
    set_tests_properties("test_${TEST}" PROPERTIES TIMEOUT 60)
  endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
endif()
