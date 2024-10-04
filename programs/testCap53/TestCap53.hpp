// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FM_TESTCAP53_HPP__
#define __FM_TESTCAP53_HPP__

#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <cmath>
#include <iomanip>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#include "PeriodicThreadImpl.hpp"

#define DEFAULT_ROBOT "/teo"    // Por defecto, usaremos el robot // -> con simulador: "/teoSim"

namespace roboticslab
{

/**
 * @ingroup testCap53
 *
 * @brief testCap53.
 *
 */
class TestCap53 : public RFModule {
private:
    PeriodicThreadImpl * periodicThreadImpl {nullptr};

    yarp::os::Port portft0;
    yarp::os::Port portft1;

    /** Axes number **/
    int numLeftLegJoints;
    /** Left Leg Device */
    yarp::dev::PolyDriver leftLegDevice;
    /** Encoders **/
    yarp::dev::IEncoders *leftLegIEncoders;
    /** Left Leg ControlMode Interface */
    yarp::dev::IControlMode *leftLegIControlMode;
    /** Left Leg PositionDirect Interface */
    yarp::dev::IPositionDirect *leftLegIPositionDirect; // para control en posicion

    /** Axes number **/
    int numRightLegJoints;
    /** Right Leg Device */
    yarp::dev::PolyDriver rightLegDevice;
    /** Encoders **/
    yarp::dev::IEncoders *rightLegIEncoders;
    /** Right Leg ControlMode Interface */
    yarp::dev::IControlMode *rightLegIControlMode;
    /** Right Leg PositionDirect Interface */
    yarp::dev::IPositionDirect *rightLegIPositionDirect; // para control en posicion

    // -------------------------------------------------------------------------------------

    bool interruptModule();
    double getPeriod();
    bool updateModule();

public:
    bool configure(ResourceFinder &rf) override;
    bool close() override;
};

}  // namespace roboticslab

#endif // __FM_TESTCAP53_HPP__
