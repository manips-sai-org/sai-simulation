# License pending
# Original author: seb
# Subsequent authors: Shameek Ganguly


# common settings required to build examples
if (CMAKE_SYSTEM_NAME MATCHES Windows)
  find_package (FreeGLUT REQUIRED)
  include_directories (${FREEGLUT_INCLUDE_DIRS})
  set (GLUT_LIBRARIES ${FREEGLUT_LIBRARIES})
else ()
  find_package (GLUT REQUIRED)
endif ()

# build all targets
foreach (example 01-blocks 02-peg 05-rover)

  file (GLOB source ${example}/*.cpp)
  add_executable (${example} ${source})
  target_link_libraries (${example} ${SAI-SIMULATION_LIBRARIES} ${CHAI3D_LIBRARIES} ${GLUT_LIBRARIES})

endforeach ()
