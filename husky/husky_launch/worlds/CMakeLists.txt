cmake_minimum_required(VERSION 2.8.3)
project(husky_launch)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)
