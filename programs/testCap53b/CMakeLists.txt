option(ENABLE_testCap53b "Enable/disable testCap53b program" ON)

if(ENABLE_testCap53b)

    find_package(fcontrol REQUIRED)

    # Same as YARP, overrides fcontrol's transitive C++11 standard.
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    add_executable(TEO_identification main.cpp
                                      global.h
                                      TestCap53b.hpp
                                      TestCap53b.cpp
                                      ThreadImpl.hpp
                                      ThreadImpl.cpp)

    target_link_libraries(TEO_identification YARP::YARP_init
                                             YARP::YARP_os
                                             YARP::YARP_dev
                                             ROBOTICSLAB::ColorDebug
                                             ROBOTICSLAB::KinematicRepresentationLib
                                             ROBOTICSLAB::KinematicRepresentationLib
                                             fcontrol)

    install(TARGETS TEO_identification)

endif(ENABLE_testCap53b)
