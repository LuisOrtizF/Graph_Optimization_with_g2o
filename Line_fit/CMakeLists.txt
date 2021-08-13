cmake_minimum_required( VERSION 3.5.1)
set(execName line_fit)
project(${execName})

# Set C++11 as default
set( CMAKE_CXX_FLAGS "-std=c++11" )

# Add *.cmake files in g2o cmake_modules dir
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Find required libs
find_package( G2O REQUIRED )
message(STATUS "G2O Found: " ${G2O_FOUND})

find_package( Eigen3 REQUIRED )
message(STATUS "Eigen3 Found: " ${EIGEN3_FOUND})

find_package( CSparse REQUIRED )
message(STATUS "CSPARSE Found: " ${CSPARSE_FOUND})

include_directories( ${EIGEN3_INCLUDE_DIR} ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

LIST(APPEND G2O_LIBS
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_CLI_LIBRARY} ${G2O_SOLVER_CHOLMOD}
    ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_SOLVER_STRUCTURE_ONLY}
    ${G2O_SOLVER_EIGEN} ${G2O_TYPES_DATA} ${G2O_TYPES_ICP} ${G2O_TYPES_SBA}
    ${G2O_TYPES_SCLAM2D} ${G2O_TYPES_SIM3} ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D}
    ${CSPARSE_LIBRARY}
)

add_executable( ${execName}  main.cpp )

target_link_libraries( ${execName} ${G2O_LIBS} )
