option(ENABLE_testCap81 "Enable/disable testCap81 program" OFF)

if(ENABLE_testCap81)

    add_executable(bodyControl main.cpp
                               MyRateThread.hpp
                               MyRateThread.cpp)

    target_link_libraries(bodyControl YARP::YARP_init
                                      YARP::YARP_os
                                      YARP::YARP_dev
                                      ROBOTICSLAB::ColorDebug
                                      ROBOTICSLAB::KinematicsDynamicsInterfaces)

    install(TARGETS bodyControl)

endif(ENABLE_testCap81)
