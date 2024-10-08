// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PERIODIC_THREAD_IMPL_HPP__
#define __PERIODIC_THREAD_IMPL_HPP__

#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include <deque>
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

#include "LIPM2d.h"
#include "global.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

static FILE *fp;

namespace roboticslab
{

/**
 * @ingroup PeriodicThreadImpl
 *
 * @brief Input port of Force/Torque data.
 *
 */
class PeriodicThreadImpl : public yarp::os::PeriodicThread
{
public:

    PeriodicThreadImpl(double period) : PeriodicThread(period, yarp::os::PeriodicThreadClock::Absolute) { }

    void setIEncodersControl(IEncoders *iRightLegEncoders, IEncoders *iLeftLegEncoders);
    void setIPositionDirect(IPositionDirect *iRightLegPositionControl, IPositionDirect *iLeftLegPositionControl);
    void setInputPorts(yarp::os::Port *inputPortFt0, yarp::os::Port *inputPortFt1);

protected:

    //-- IMU variables
    double acc_x, acc_y, acc_z;
    double ang_x, ang_y, ang_z;
    double spd_x, spd_y, spd_z;
    double mag_x, mag_y, mag_z;
    deque<double> x_sensor, y_sensor, z_sensor;
    //-- IMU LOW-FILTER variables & CONVERTION
    double ddx, ddy, ddz, ddx_robot, ddy_robot, ddz_robot;

    //-- struct variable for F/T reading from the JR3 sensors
    struct SensorFT_Data {
        struct ForceVector {
            double fx, fy, fz;
        } _F; // force vector by JR3
        struct TorqueVector {
            double mx, my, mz;
        } _T; // torque vector by JR3
    } _LH, _RH, _LF, _RF, _med;

    int a, b, n; // main process control variables

    //-- FT & IMU LOW-FILTER variables
    float offs_x_imu, offs_x_ft; // zmp offset in initial time - frontal plane
    float offs_y_imu, offs_y_ft; // zmp offset in initial time - frontal plane
    float sum_x_imu, sum_y_imu, sum_x_ft, sum_y_ft; // adding offset values.

    //-- ZMP & IMU variables
    float _xzmp_ft0, _yzmp_ft0; // ZMP-FT sensor 0 (right)
    float _xzmp_ft1, _yzmp_ft1; // ZMP-FT sensor 1 (left)
    float _xzmp_ft01, _yzmp_ft01; // ZMP double support
    float Xzmp_ft, Yzmp_ft; // Global ZMP-FT after filter

    //-- Control/Movement variables
    float zmp_ref, _ang_ref, _ang_out, ka;

    //-- Time variables
    double init_time, act_time, init_loop, act_loop, it_time, it_prev; // for calculating process time

    LIPM2d _evalLIPM; // Discrete-time Space State DLIPM Model evaluation

    //-- Sensors variables
    yarp::os::Port *portFt0;
    yarp::os::Port *portFt1;

    //-- PeriodicThreadImpl Funtions
    bool threadInit();
    void run();

    void confCSVfile();/** Configuring CSV file **/
    void openingPorts();/** Opening Ports & connecting with sensor programs **/

    void readSensorsFT0();/** Reading from the FT0_JR3_sensor. **/
    void readSensorsFT1();/** Reading from the FT1_JR3_sensor. **/

    void zmpCompFT();/** Calculating ZMP-FT of the body. **/
    void evaluateModel();/** Calculating OUTPUT (Qi) of the legs. **/
    void setJoints();/** Position control. **/

    void printData();/** Printing data info on terminal **/
    void saveInFileCsv();/** Saving the ZMP measurements. **/

    void getInitialTime();
    void getCurrentTime();

    /** Axes number **/
    int numtLegJoints;
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
};

} // namespace roboticslab

#endif // __PERIODIC_THREAD_IMPL_HPP__
