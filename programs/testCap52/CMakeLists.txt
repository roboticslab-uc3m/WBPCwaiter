option(ENABLE_testCap52 "Enable/disable testCap52 program" ON)

if(ENABLE_testCap52)

    add_executable(3dslope_threat main.cpp
                                  global.h
                                  TestCap52.hpp
                                  TestCap52.cpp
                                  ThreadImpl.hpp
                                  ThreadImpl.cpp)

    target_link_libraries(3dslope_threat YARP::YARP_init
                                         YARP::YARP_os
                                         YARP::YARP_dev
                                         ROBOTICSLAB::ColorDebug
                                         ROBOTICSLAB::KinematicsDynamicsInterfaces
                                         ROBOTICSLAB::KinematicRepresentationLib)

    install(TARGETS 3dslope_threat)

endif(ENABLE_testCap52)
