option(ENABLE_testCap53a "Enable/disable testCap53a program" ON)

if(ENABLE_testCap53a)

    add_executable(LOLI_test main.cpp
                             global.h
                             ratethread.h
                             TestCap53a.hpp
                             TestCap53a.cpp
                             ThreadImpl.hpp
                             ThreadImpl.cpp
                             LIPM2d.h
                             LIPM2d.cpp)

    target_link_libraries(LOLI_test YARP::YARP_init
                                    YARP::YARP_os
                                    YARP::YARP_dev
                                    ROBOTICSLAB::ColorDebug
                                    ROBOTICSLAB::KinematicsDynamicsInterfaces
                                    ROBOTICSLAB::KinematicRepresentationLib)

    install(TARGETS LOLI_test)

endif(ENABLE_testCap53a)
