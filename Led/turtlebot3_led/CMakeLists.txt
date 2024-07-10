cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_led)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(
    FILES 
    choisirCouleur.srv
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    )

    