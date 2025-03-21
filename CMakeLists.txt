cmake_minimum_required(VERSION 3.15.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(door_monitoring)

target_sources(app PRIVATE src/main.c src/application/door_monitoring.c)

target_include_directories(app PRIVATE include/application)

#####################################################################
# not working

# cmake_minimum_required(VERSION 3.15.0)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# project(door_monitoring)

# # Add the library directory
# add_subdirectory(src/application)

# # Add main application source file
# target_sources(app PRIVATE src/main.c)

# # Include directories for the application
# target_include_directories(app PRIVATE include/application)





