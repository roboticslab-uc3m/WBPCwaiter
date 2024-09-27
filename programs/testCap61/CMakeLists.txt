option(ENABLE_testCap61 "Enable/disable testCap61 program" OFF)

if(ENABLE_testCap61)

    add_executable(carttable_threat main.cpp
                                    global.h
                                    TestCap61.hpp
                                    TestCap61.cpp
                                    ThreadImpl.hpp
                                    ThreadImpl.cpp
                                    LIPM2d.h
                                    LIPM2d.cpp
                                    MyRateThread.hpp
                                    MyRateThread.cpp)

    target_link_libraries(carttable_threat YARP::YARP_init
                                           YARP::YARP_os
                                           YARP::YARP_dev
                                           ROBOTICSLAB::ColorDebug
                                           ROBOTICSLAB::KinematicsDynamicsInterfaces
                                           ROBOTICSLAB::KinematicRepresentationLib)

    install(TARGETS carttable_threat)

endif(ENABLE_testCap61)
