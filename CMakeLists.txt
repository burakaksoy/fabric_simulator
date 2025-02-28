cmake_minimum_required(VERSION 3.0.2)
project(fabric_simulator)

# colors, because why not :)
string(ASCII 27 Esc)
set(RED "${Esc}[31m")
set(GREEN "${Esc}[32m")
set(ENDL "${Esc}[m")

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")
# or
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# or
# set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall")

MESSAGE("${GREEN}----------------Options:-------------------${ENDL}")
MESSAGE("${GREEN}Build type: ${CMAKE_BUILD_TYPE}${ENDL}")

set(MSG_DEPS
    geometry_msgs
    std_msgs
    #   visualization_msgs
)

option(BUILD_WITH_OPENMP "Use OpenMP" ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  visualization_msgs
  message_generation
  ${MSG_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system chrono)
find_package(Eigen3)

# Print eigen3 include directory and version
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Eigen3 version: ${EIGEN3_VERSION_STRING}")

if(BUILD_WITH_OPENMP)
    find_package(OpenMP)
    if(NOT OpenMP_FOUND)
        message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
    else()
        message(STATUS "OPENMP FOUND")
        # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
        # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
        set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
        set(OpenMP_LIBRARIES gomp)
    endif()
endif()

## Discregrid Integration
# This section is required for generating 3D object file signed distance fields (SDF) using Discregrid.
if(NOT DEFINED Discregrid_INCLUDE_DIR OR NOT DEFINED Discregrid_LIBRARIES)
    # If Discregrid paths are not predefined, set up Discregrid as an external project.

    include(ExternalProject)
    ExternalProject_Add(
        Ext_Discregrid
        PREFIX "${CMAKE_BINARY_DIR}/extern/Discregrid"
        GIT_REPOSITORY https://github.com/InteractiveComputerGraphics/Discregrid.git
        # Consider specifying a specific tag or commit to ensure consistent builds.
        GIT_TAG "master"  # Or replace with a specific commit like "4c27e1cc88be828c6ac5b8a05759ac7e01cf79e9"
        INSTALL_DIR "${CMAKE_BINARY_DIR}/extern/Discregrid"
        CMAKE_ARGS 
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
            -DBUILD_AS_SHARED_LIBS=ON
            -DBUILD_CMD_EXECUTABLE:BOOL=0
            -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/extern/Discregrid
            -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
            -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    )

    # Retrieve and cache the installation directory of Discregrid.
    ExternalProject_Get_Property(Ext_Discregrid INSTALL_DIR)
    set(Discregrid_INCLUDE_DIR "${INSTALL_DIR}/include" CACHE INTERNAL "Discregrid include directory")
    set(Discregrid_LIBRARIES "${INSTALL_DIR}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}Discregrid${CMAKE_SHARED_LIBRARY_SUFFIX}" CACHE INTERNAL "Discregrid libraries")
else()
    # If Discregrid paths are predefined, use them.
    message(STATUS "Using pre-defined Discregrid: ${Discregrid_INCLUDE_DIR}")
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

set(MSG_FILES 
    MinDistanceData.msg
    MinDistanceDataArray.msg
    ChangeParticleDynamicity.msg
    SegmentState.msg
    SegmentStateArray.msg
    AttachExternalOdomFrameRequest.msg
    FixNearestFabricParticleRequest.msg
    SyncParticleUpdateData.msg
    SyncParticleUpdateDataArray.msg
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ${MSG_FILES}
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    EnableCollisionHandling.srv
    GetFabricBendingCompliance.srv
    GetFabricStretchingCompliance.srv
    SetFabricBendingCompliance.srv
    SetFabricStretchingCompliance.srv
    SetParticleDynamicity.srv
    AttachExternalOdomFrame.srv
    FixNearestFabricParticle.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ${MSG_DEPS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
    fabric_simulator # cloth, maybe?
 CATKIN_DEPENDS 
    geometry_msgs 
    roscpp 
    rospy 
    std_msgs 
    tf tf2 tf2_ros 
    visualization_msgs 
    eigen_conversions 
    message_runtime 
    ${MSG_DEPS} 
 DEPENDS 
    EIGEN3 # OpenMP, maybe?
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenMP_INCLUDE_DIRS})
include_directories(${Discregrid_INCLUDE_DIR})

#
# Build libs
#

## MEANINGS OF SOME OF THE COMMANDS 

## add_libray: this command is used to create a library from some source files. In case below, add_library(fabric_simulator src/fabric_simulator.cpp src/cloth.cpp) is creating a library named fabric_simulator from the source files fabric_simulator.cpp and cloth.cpp. This library can be linked to executables or other libraries.

## target_link_libraries: This command is used to specify libraries to link against. In case below, target_link_libraries(fabric_simulator ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_LIBRARIES}) is linking the fabric_simulator library with other libraries, such as catkin_LIBRARIES, Boost_LIBRARIES, and OpenMP_LIBRARIES. This means that fabric_simulator depends on these libraries.

## add_dependencies: This command adds a dependency to a target. add_dependencies(fabric_simulator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}) ensures that the specified targets (catkin_EXPORTED_TARGETS and ${PROJECT_NAME}_EXPORTED_TARGETS) are built before fabric_simulator. This is especially important in projects where multiple targets are dependent on each other.

## add_executable: This command creates an executable from the specified source files. add_executable(fabric_simulator_node src/nodes/fabric_simulator_node.cpp) is creating an executable named fabric_simulator_node from the source file fabric_simulator_node.cpp.

add_library(fabric_simulator 
    src/fabric_simulator/fabric_simulator.cpp 
    src/fabric_simulator/utilities/cloth.cpp
    src/fabric_simulator/utilities/rigid_body_scene_loader.cpp 
    src/fabric_simulator/utilities/collision_handler.cpp
    src/fabric_simulator/utilities/BoundingSphereHierarchy.cpp
) 
target_link_libraries(fabric_simulator 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${OpenMP_LIBRARIES}
    ${Discregrid_LIBRARIES}
)
# Standard dependencies
add_dependencies(fabric_simulator 
    ${catkin_EXPORTED_TARGETS} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Add Ext_Discregrid as a dependency only if it exists
if(TARGET Ext_Discregrid)
    add_dependencies(fabric_simulator Ext_Discregrid)
endif()

#
# Build nodes
#

add_executable(fabric_simulator_node 
    src/nodes/fabric_simulator_node.cpp
)
target_link_libraries(fabric_simulator_node 
    ${catkin_LIBRARIES} 
    fabric_simulator
)


# #############
# ## Install ##
# #############

# #
# # Install libraries
# #
# ## Mark libraries for installation
# ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS fabric_simulator # cloth?
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



# #
# # Install header files
# #
# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   # FILES_MATCHING PATTERN "*.h" maybe?
# )



# #
# # Install nodes
# #
# ## Mark executables for installation
# ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS fabric_simulator_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



# #
# # Install launch files
# #
# # install(DIRECTORY launch/
# #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




# # all install targets should use catkin DESTINATION variables
# # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# ## Mark executable scripts (Python etc.) for installation
# ## in contrast to setup.py, you can choose the destination
# # catkin_install_python(PROGRAMS
# #   scripts/my_python_script
# #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# # )




# ## Mark other files for installation (e.g. launch and bag files, etc.)
# # install(FILES
# #   # myfile1
# #   # myfile2
# #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# # )