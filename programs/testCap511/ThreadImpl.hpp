// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __THREAD_IMPL_HPP__
#define __THREAD_IMPL_HPP__

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
#include "ColorDebug.h"
#include <iomanip>

#include "KinematicRepresentation.hpp"
#include "ICartesianSolver.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace roboticslab;

#define DEFAULT_QDOT_LIMIT 10
#define DEFAULT_RATE_MS 20

#define DEFAULT_STRATEGY "positionDirect"
//#define DEFAULT_STRATEGY "velocity"

static FILE *fp;

namespace roboticslab
{

/**
 * @ingroup ThreadImpl
 *
 * @brief Input port of Force/Torque data.
 *
 */
class ThreadImpl : public yarp::os::Thread {
    public:

        void setIEncodersControl(yarp::dev::IEncoders *iEncoders) {
            this->leftArmIEncoders = iEncoders;        }

        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->leftArmIPositionControl = iPositionControl;        }

        void setIVelocityControl(yarp::dev::IVelocityControl *iVelocityControl) {
            this->leftArmIVelocityControl = iVelocityControl;        }

        void setICartesianSolver(roboticslab::ICartesianSolver *iCartesianSolver) {
            this->leftArmICartesianSolver = iCartesianSolver;        }

        void setInputPorts(yarp::os::Port *inputPortImu, yarp::os::Port *inputPortFt0, yarp::os::Port *inputPortFt1, yarp::os::Port *inputPortFt2, yarp::os::Port *inputPortFt3) {
            this->portImu = inputPortImu;
            this->portFt0 = inputPortFt0;
            this->portFt1 = inputPortFt1;
            this->portFt2 = inputPortFt2;
            this->portFt3 = inputPortFt3;        }

    protected:

    /*    // ----- variables para el control del cuerpo -----
        //-- IMU variables
        double acc_x, acc_y, acc_z, ang_x, ang_y, ang_z, spd_x, spd_y, spd_z, mag_x, mag_y, mag_z; // IMU inputs
        //-- IMU LOW-FILTER variables & CONVERTION
        double ddx, ddy, ddz, ddx_robot, ddy_robot, ddz_robot; // additional acc variables
        std::deque<double> x_sensor, y_sensor, z_sensor;

        //-- FT variables
        float _fx0, _fy0, _fz0, _mx0, _my0, _mz0; // F-T from sensor 0 [Fuerza en N y Pares en Nm*10]
        float _fx1, _fy1, _fz1, _mx1, _my1, _mz1; // F-T from sensor 1 [Fuerza en N y Pares en Nm*10]
        //-- FT LOW-FILTER variables
        float offs_x_j, offs_x_l, offs_x_l2; // zmp offset in initial time.
        float offs_y;
        float sum_j, sum_l, sum_l2;

        //-- ZMP variables
        float _xzmp0_ft, _yzmp0_ft; // ZMP sensor 0 (derecho)
        float _xzmp1_ft, _yzmp1_ft; // ZMP sensor 1 (izquierdo)
        float _xzmp01_ft, _yzmp01_ft; // ZMP robot (zmp_0 + zmp_0) metros
        float Xzmp_ft, Yzmp_ft; // Global ZMP-FT despues de filtrar
        double Xzmp_imu, Yzmp_imu, Xzmp_total; // Global ZMP-IMU despues de filtrar*/

        // ----- variables para el control de la Bandeja -----
        struct SensorData { // lectura F/T del sensor JR3
            struct ForceVector {
                double fx, fy, fz;
            } _initF; // vector de fuerza proporcionado por el JR3
            struct TorqueVector {
                double mx, my, mz;
            } _initT; // vector de momento proporcionado por el JR3
        } _FTLeftHand, _FTRightHand, _FTLeftFoot, _FTRightFoot, _med;

        struct TrayData { // variable para el control con los valores finales en la bandeja
            struct ForceVector {
                double fx, fy, fz;
            } _F; // vector de fuerza despues de la transformacion de orientacion
            struct TorqueVector {
                double mx, my, mz;
            } _M; // vector de momento despues de la transformacion de orientacion
            struct ZMPVector {
                double x_zmp, y_zmp;
            } _zmp; // vector ZMP con sus dos componentes X e Y
        } _tray, _off;

        int a, b, w, n; // control porcess variables
        int inputAngle; // angle input to move the trayTCP (openloop)
        int pepinito;   // no used on test51 (void mediumJR3(Bottle& FTsensor))

        float _d, _l;  // distance in metres between the SDC tray and the SDC jr3

        double _rzmp, _rzmp_b, _rWorkSpace, _rFxy, _modFS, _modFF, x_zmp_b, y_zmp_b;
        double angle, _thetaX, _thetaY, _thetaXX, _thetaYY;

        std::vector<double> quat, quatC, preFF, FF, preFM, FM; // quaternios - needed for transformations

        double initpos; // for preprogrammedInitTrajectory() - first joint movement
        std::vector<double> curQ, befQ, desQ; // joint space variables
        std::vector<double> befX, curX, curX_AAS, desX, desX_AAS;  // cartesian space variables

        double init_time, act_time, init_loop, act_loop, it_time, it_prev; // for calculating process time


        //-- ThreadImpl Funtions
        bool threadInit();
        void run();void offSetJR3();/** Offset JR3 measurements. **/
        void preprogrammedInitTrajectory();/** Set INITIAL POS-VEL-ACC **/
        void getInitialTime();
        void readSensors();/** Reading from the FT_JR3_sensor. **/
        void axesTransform1();/** Rotation Transformation matrix of JR3. **/
        void axesTransform2();/** Transformation matrix between JR3 and tray. **/
        void zmpComp();/** Calculating ZMP of the bottle. **/
        void LIPM3d();/** Control based on the 3D-LIMP. **/
        void printData();
        void saveInFileCsv();/** Saving the ZMP measurements. **/
        void getCurrentTime();

        yarp::os::Port *portFt0;
        yarp::os::Port *portFt1;
        yarp::os::Port *portFt2;
        yarp::os::Port *portFt3;
        yarp::os::Port *portImu;

        //yarp::os::Port *portft3;


        /** Axes number **/
        int numHeadJoints;
        /** Head Device */
        yarp::dev::PolyDriver headDevice;
        /** Encoders **/
        yarp::dev::IEncoders *headIEncoders;
        /** Head ControlMode Interface */
        yarp::dev::IControlMode *headIControlMode;
        /** Head PositionControl Interface */
        yarp::dev::IPositionControl *headIPositionControl; // para control en posicion
        /** Head VelocityControl Interface */
        yarp::dev::IVelocityControl *headIVelocityControl; // para control en velocidad

        /** Axes number **/
        int numLeftArmJoints;
        /** Left Arm Device */
        yarp::dev::PolyDriver leftArmDevice;
        /** Encoders **/
        yarp::dev::IEncoders *leftArmIEncoders;
        /** Left Arm ControlMode Interface */
        yarp::dev::IControlMode *leftArmIControlMode;
        /** Left Arm PositionControl Interface */
        yarp::dev::IPositionControl *leftArmIPositionControl; // para control en posicion
        /** Left Arm VelocityControl Interface */
        yarp::dev::IVelocityControl *leftArmIVelocityControl; // para control en velocidad

        /** Axes number **/
        int numRightArmJoints;
        /** Right Arm Device */
        yarp::dev::PolyDriver rightArmDevice;
        /** Encoders **/
        yarp::dev::IEncoders *rightArmIEncoders;
        /** Right Arm ControlMode Interface */
        yarp::dev::IControlMode *rightArmIControlMode;
        /** Right Arm PositionControl Interface */
        yarp::dev::IPositionControl *rightArmIPositionControl; // para control en posicion
        /** Right Arm VelocityControl Interface */
        yarp::dev::IVelocityControl *rightArmIVelocityControl; // para control en velocidad

        // ----------------------------------------------------------------------------------

        /** Axes number **/
        int numTrunkJoints;
        /** Trunk Device */
        yarp::dev::PolyDriver trunkDevice;
        /** Encoders **/
        yarp::dev::IEncoders *trunkIEncoders;
        /** Trunk ControlMode Interface */
        yarp::dev::IControlMode *trunkIControlMode;
        /** Trunk PositionControl Interface */
        yarp::dev::IPositionControl *trunkIPositionControl; // para control en posicion
        /** Trunk VelocityControl Interface */
        yarp::dev::IVelocityControl *trunkIVelocityControl; // para control en velocidad

        /** Axes number **/
        int numtLegJoints;
        /** Left Leg Device */
        yarp::dev::PolyDriver leftLegDevice;
        /** Encoders **/
        yarp::dev::IEncoders *leftLegIEncoders;
        /** Left Leg ControlMode Interface */
        yarp::dev::IControlMode *leftLegIControlMode;
        /** Left Leg PositionControl Interface */
        yarp::dev::IPositionControl *leftLegIPositionControl; // para control en posicion
        /** Left Leg VelocityControl Interface */
        yarp::dev::IVelocityControl *leftLegIVelocityControl; // para control en velocidad

        /** Axes number **/
        int numRightLegJoints;
        /** Right Leg Device */
        yarp::dev::PolyDriver rightLegDevice;
        /** Encoders **/
        yarp::dev::IEncoders *rightLegIEncoders;
        /** Right Leg ControlMode Interface */
        yarp::dev::IControlMode *rightLegIControlMode;
        /** Right Leg PositionControl Interface */
        yarp::dev::IPositionControl *rightLegIPositionControl; // para control en posicion
        /** Right Leg VelocityControl Interface */
        yarp::dev::IVelocityControl *rightLegIVelocityControl; // para control en velocidad

        // -------------------------------------------------------------------------------------

        /** Lelt Arm ControlLimits2 Interface */
        yarp::dev::IControlLimits *leftArmIControlLimits;
        /** Solver device **/
        yarp::dev::PolyDriver leftArmSolverDevice;
        roboticslab::ICartesianSolver *leftArmICartesianSolver;
        /** Forward Kinematic function **/
        bool getleftArmFwdKin(std::vector<double> *currentX);

        /** Right Arm ControlLimits2 Interface */
        yarp::dev::IControlLimits *rightArmIControlLimits;
        /** Solver device **/
        yarp::dev::PolyDriver rightArmSolverDevice;
        roboticslab::ICartesianSolver *rightArmICartesianSolver;
        /** Forward Kinematic function **/
        bool getRightArmFwdKin(std::vector<double> *currentX);

};

}  // namespace roboticslab

#endif // __THREAD_IMPL_HPP__
