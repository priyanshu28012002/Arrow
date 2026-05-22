cmake_minimum_required(VERSION 3.16)
project(serviceAction CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
add_compile_options(
    -Wall -Wextra -Wpedantic
    -O2
    -pthread
)

# ── Common include path ───────────────────────────────────────────────────────
include_directories(${CMAKE_SOURCE_DIR})

# ── Combined examples (server+client in one process) ─────────────────────────
add_executable(local_service_example examples/main1.cpp)
target_link_libraries(local_service_example pthread rt)
add_executable(local_action_example examples/main2.cpp)
target_link_libraries(local_action_example pthread rt)
add_executable(local_stream_example examples/main3.cpp)
target_link_libraries(local_stream_example pthread rt)
add_executable(network_service_example examples/main4.cpp)
target_link_libraries(network_service_example pthread rt)
add_executable(network_action_example examples/main5.cpp)
target_link_libraries(network_action_example pthread rt)
add_executable(network_stream_example examples/main6.cpp)
target_link_libraries(network_stream_example pthread rt)


add_executable(network_service_client examples/network/service/control_client.cpp)
target_link_libraries(network_service_client pthread rt)

add_executable(network_service_server examples/network/service/robot_server.cpp)
target_link_libraries(network_service_server pthread rt)


add_executable(network_action_client examples/network/action/control_client.cpp)
target_link_libraries(network_action_client pthread rt)

add_executable(network_action_server examples/network/action/robot_server.cpp)
target_link_libraries(network_action_server pthread rt)


# Connection 

# add_executable(robot_server examples/connection/robot_server.cpp)
# target_link_libraries(robot_server pthread rt)

# add_executable(control_client examples/connection/control_client.cpp)
# target_link_libraries(control_client pthread rt)


add_executable(robot_watchdog /home/octobot/Github/ServiceActions/Arrow/examples/network_wrapper_robot_end/robot.cpp)
target_link_libraries(robot_watchdog pthread rt)

add_executable(client_watchdog /home/octobot/Github/ServiceActions/Arrow/examples/network_wrapper_ui_end/client.cpp)
target_link_libraries(client_watchdog pthread rt)

# ── Output all binaries to build/ ─────────────────────────────────────────────
set_target_properties(
    local_service_example
     local_action_example
     local_stream_example
     network_service_example
     network_action_example
     network_stream_example
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
