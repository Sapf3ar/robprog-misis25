cmake_minimum_required(VERSION 3.8)
project(bt_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
	  bt_demo/bt_executor_node.py
	    DESTINATION lib/${PROJECT_NAME}
	    RENAME bt_executor_node
	    )
install(DIRECTORY launch
	      DESTINATION share/${PROJECT_NAME}
	      )
if(BUILD_TESTIN)
	find_package(launch_testing_ament_cmake REQUIRED)
	add_launch_test(test/test_bt_integration.test.py)
endif()
	ament_package()

