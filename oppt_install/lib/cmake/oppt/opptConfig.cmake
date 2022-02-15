# ...
 # (compute PREFIX relative to file location)
 # ...

include("${CMAKE_CURRENT_LIST_DIR}/opptTargets.cmake")

set(oppt_INCLUDE_DIRS /home/rdl/comp3770/hmi-twoaxis-simplatform/oppt_install/include;/usr/include;/usr/include/gazebo-9;/usr/include/bullet;/usr/include/simbody;/usr/include;/usr/include;/usr/include/sdformat-6.0;/usr/include/ignition/math4;/usr/include;/usr/include/OGRE;/usr/include;/usr/include;/usr/include/OGRE/Terrain;/usr/include/OGRE/Paging;/usr/include/ignition/math4;/usr/include/ignition/transport4;/usr/include/ignition/msgs1;/usr/include/ignition/common1;/usr/include/ignition/fuel_tools1;/usr/include/gazebo-9/gazebo;/usr/include;/usr/include/gazebo-9;/usr/include/bullet;/usr/include/simbody;/usr/include;/usr/include;/usr/include/sdformat-6.0;/usr/include/ignition/math4;/usr/include;/usr/include/OGRE;/usr/include;/usr/include;/usr/include/OGRE/Terrain;/usr/include/OGRE/Paging;/usr/include/ignition/math4;/usr/include/ignition/transport4;/usr/include/ignition/msgs1;/usr/include/ignition/common1;/usr/include/ignition/fuel_tools1;/usr/include/gazebo-9/gazebo;;/usr/local/include/spatialindex;/usr/include/sdformat-6.0;/usr/include/ignition/math4;/usr/include;/usr/include/eigen3;/usr/include;;/opt/ros/melodic/include;/usr/include;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;/opt/ros/melodic/include;/opt/ros/melodic/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3;/usr/include;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;/opt/ros/melodic/include;/usr/include;/opt/ros/melodic/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;/usr/include)
set(oppt_INCLUDE_TARGETS /usr/include;/usr/include/gazebo-9;/usr/include/bullet;/usr/include/simbody;/usr/include;/usr/include;/usr/include/sdformat-6.0;/usr/include/ignition/math4;/usr/include;/usr/include/OGRE;/usr/include;/usr/include;/usr/include/OGRE/Terrain;/usr/include/OGRE/Paging;/usr/include/ignition/math4;/usr/include/ignition/transport4;/usr/include/ignition/msgs1;/usr/include/ignition/common1;/usr/include/ignition/fuel_tools1;/usr/include/gazebo-9/gazebo;;/usr/local/include/spatialindex;/usr/include/sdformat-6.0;/usr/include/ignition/math4;/usr/include;/usr/include/eigen3;/usr/include;;/opt/ros/melodic/include;/usr/include;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;/opt/ros/melodic/include;/opt/ros/melodic/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3;/usr/include;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;/opt/ros/melodic/include;/usr/include;/opt/ros/melodic/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;/usr/include)
set(oppt_LIBRARIES oppt)
#set(oppt_LIBRARY_DIRS )

set(GZ_GT_7 True)
if (GZ_GT_7)
  add_definitions(-DGZ_GT_7)
endif ()

set(GZ_GT_10 False)
if (GZ_GT_10)
  add_definitions(-DGZ_GT_10)
endif ()

add_definitions(-DUSE_DOUBLE_PRECISION=true)

include("${CMAKE_CURRENT_LIST_DIR}/opptMacros.cmake")

## Required, otherwise some neccessary ignition macros are not invoked
find_package(gazebo REQUIRED)

## Required on Ubuntu 20.04
find_package(Boost
             REQUIRED 
             system 
             thread             
             filesystem 
             serialization)
