# name of the example
set(EXAMPLE_NAME 02-fixed_joint)

# create an executable
add_executable (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
target_link_libraries (${EXAMPLE_NAME} ${SAI-SIMULATION_EXAMPLES_LIBRARIES})