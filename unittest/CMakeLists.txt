if(CATKIN_ENABLE_TESTING)
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
    test_solver_queue
    ${catkin_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
    ${pybind11_catkin_LIBRARIES}
    ${Python3_LIBRARIES}
    ${Boost_PYTHON_LIBRARY})

  # -----------------------
  # Python Tests (ROS1)
  # -----------------------
  set(${PROJECT_NAME}_PYTHON_TESTS solver_statistics solver_trajectory)

  foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
    add_rostest(crocoddyl_msgs.test ARGS test_name:=${TEST})
  endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

else()  # ROS2
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
  endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
endif()
