option(ENABLE_testCap511 "Enable/disable testCap511 program" ON)

if(ENABLE_testCap511)

    add_executable(force_T1_threat main.cpp
                                   global.h
                                   TestCap511.hpp
                                   TestCap511.cpp
                                   ThreadImpl.hpp
                                   ThreadImpl.cpp)

    target_link_libraries(force_T1_threat YARP::YARP_init
                                         YARP::YARP_os
                                         YARP::YARP_dev
                                         ROBOTICSLAB::ColorDebug
                                         ROBOTICSLAB::KinematicsDynamicsInterfaces
                                         ROBOTICSLAB::KinematicRepresentationLib)

    install(TARGETS force_T1_threat)

endif(ENABLE_testCap511)
