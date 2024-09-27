option(ENABLE_testCap10 "Enable/disable testCap10 program" OFF)

if(ENABLE_testCap10)

    add_executable(wholeBalance main.cpp
                                MyRateThread.hpp
                                MyRateThread.cpp)

    target_link_libraries(wholeBalance YARP::YARP_init
                                       YARP::YARP_os
                                       YARP::YARP_dev
                                       ROBOTICSLAB::ColorDebug
                                       ROBOTICSLAB::KinematicsDynamicsInterfaces)

    install(TARGETS wholeBalance)

endif(ENABLE_testCap10)
