option(ENABLE_testCap54 "Enable/disable testCap54 program" ON)

if(ENABLE_testCap54)

    add_executable(dlipm_threat main.cpp
                                global.h
                                TestCap54.hpp
                                TestCap54.cpp
                                ThreadImpl.hpp
                                ThreadImpl.cpp
                                LIPM2d.h
                                LIPM2d.cpp)

    target_link_libraries(dlipm_threat YARP::YARP_init
                                       YARP::YARP_os
                                       YARP::YARP_dev
                                       ROBOTICSLAB::ColorDebug
                                       ROBOTICSLAB::KinematicsDynamicsInterfaces
                                       ROBOTICSLAB::KinematicRepresentationLib)

    install(TARGETS dlipm_threat)

endif(ENABLE_testCap54)
