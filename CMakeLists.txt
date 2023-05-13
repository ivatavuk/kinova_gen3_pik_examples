cmake_minimum_required(VERSION 2.8.3)
project(kinova_gen3_pik_examples)

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PtscEigen REQUIRED)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra)

## Find catkin and any catkin packages
find_package( catkin REQUIRED COMPONENTS 
              roscpp rospy std_msgs tf
              pik_ros)
## Declare a catkin package
catkin_package(INCLUDE_DIRS include
                DEPENDS 
                EIGEN3)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# pik_example
add_executable(pik_examples
  src/pik_examples.cpp
  src/RvizVisualization.cpp
)

target_link_libraries(pik_examples 
  PtscEigen::PtscEigen
  ${catkin_LIBRARIES}
)

# spraying_pik_example
add_executable(spraying_pik_examples
  src/spraying_pik_examples.cpp
  src/RvizVisualization.cpp
)

target_link_libraries(spraying_pik_examples
  PtscEigen::PtscEigen
  ${catkin_LIBRARIES}
)

#-----------------tests-----------------
add_executable(pik_example_1_test
  tests/pik_example_1_test.cpp
)
target_link_libraries(pik_example_1_test 
  PtscEigen::PtscEigen
  ${catkin_LIBRARIES}
)

add_executable(pik_example_2_test
  tests/pik_example_2_test.cpp
)
target_link_libraries(pik_example_2_test 
  PtscEigen::PtscEigen
  ${catkin_LIBRARIES}
)

add_executable(pik_example_3_test
  tests/pik_example_3_test.cpp
)
target_link_libraries(pik_example_3_test 
  PtscEigen::PtscEigen
  ${catkin_LIBRARIES}
)

add_executable(pik_example_4_test
  tests/pik_example_4_test.cpp
)
target_link_libraries(pik_example_4_test 
  PtscEigen::PtscEigen
  ${catkin_LIBRARIES}
)

# Define test targets
add_test(NAME pik_example_1_test COMMAND pik_example_1_test __ns:=my_gen3)
add_test(NAME pik_example_2_test COMMAND pik_example_2_test __ns:=my_gen3)
add_test(NAME pik_example_3_test COMMAND pik_example_3_test __ns:=my_gen3)
add_test(NAME pik_example_4_test COMMAND pik_example_4_test __ns:=my_gen3)