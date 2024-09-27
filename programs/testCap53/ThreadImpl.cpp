// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ThreadImpl.hpp"

namespace roboticslab
{
/************************************************************************/
bool ThreadImpl::threadInit()
{
    printf("[success] entrando en ratethread -> init/run\n");

    a = 0;
    b = 0;
    n = 1;

    zmp_ref = 0; // initial value and our zmp reference
    _ang_ref = 0; // initial value and our angle reference
    _ang_out = 0; // initial value and our output

    return true;
}

/************************************************************************/
void ThreadImpl::run()
{
    while(!isStopping()) {

        if (a!=1)    {    // STEP 1 - Creating & Configuring CSV file
            confCSVfile();
            a=1;    }
        if (a==1 && b!=1)    {    // STEP 2 - Opening & Connecting Ports
            openingPorts();
            b=1;    }

        // ------------------------------------------------------------------------
        if (a==1 && b==1)
        {
            // STEP 3 - main code

            // Test escalon con rampa // generacion del ZMP_ref para test FT sensor
            // equaction: zmp_ref = (0.0X / 30) * n - 0.X
            // where x is the ref value
            // example_1: zmp_ref = (0.05/30)*n - 0.5 ------> for zmp_ref = 0.05 meters
            // example_2: zmp_ref = (0.09/30)*n - 0.9 ------> for zmp_ref = 0.09 meters

            if (n <= 300){zmp_ref = 0.0;}
            else if (n >= 300 && n <= 330){zmp_ref = (0.05/30)*n - 0.5;} // test from 0.01 to 0.09 [m]
            else {zmp_ref = zmp_ref;}

            getInitialTime();

            readSensorsFT0();
            readSensorsFT1();
            zmpCompFT(); // calculation of the ZMP_FT

            if (n>300)  {
                evaluateModel(); // evaluacion the model and the angle output
                setJoints(); // applying the ankle movement
            }

            printData();
            cout << endl << "Press ENTER to exit..." << endl;
            cout << "*******************************" << endl << endl;
            saveInFileCsv();  // saving the information ¡
            n++;
            cout << n << endl << endl;

            getCurrentTime();
        }
    }
}

/************************************************************************/
void ThreadImpl::confCSVfile()      /** Configuring CSV file    **/
{
    cout << "[configuring] ... STEP 1 " << endl;
    fp = fopen("../data_testingDLIPM.csv","w+");
    fprintf(fp,"Time,Fx_ft0,Fz_ft0,My_ft0,Fx_ft1,Fz_ft1,My_ft1,Xzmp_ft,zmp_RF,zmp_LF,iter");
    yarp::os::Time::delay(1);

    cout << "[success] data_dlipm_threat.csv file configured." << endl;
}

/************************************************************************/
void ThreadImpl::openingPorts()     /** Opening Ports & Connecting with sensor programs **/
{
    cout << "[configuring] ... STEP 2 " << endl;

    //-- OPEN YARP PORTS
    portFt0->open("/bodyBal/jr3ch0:i");
    portFt1->open("/bodyBal/jr3ch1:i");

    //-- CONNECTIONS PORTS

    // ft right foot
    Network::connect("/jr3/ch0:o","/bodyBal/jr3ch0:i");
    if ( NetworkBase::isConnected("/jr3/ch0:o","/bodyBal/jr3ch0:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /bodyBal/jr3ch0:i." << endl;
    } else cout << "[success] Connected to /jr3ch0:i." << endl;
    yarp::os::Time::delay(0.5);
    // ft left foot
    Network::connect("/jr3/ch1:o","/bodyBal/jr3ch1:i");
    if ( NetworkBase::isConnected("/jr3/ch1:o","/bodyBal/jr3ch1:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /bodyBal/jr3ch1:i." << endl;
    } else cout << "[success] Connected to /jr3ch1:i." << endl;
    yarp::os::Time::delay(0.5);

    return;
}

/************************************************************************/
void ThreadImpl::getInitialTime()       /** Get Initial Time    **/
{
    if (n==1){init_time = Time::now();}
    init_loop = Time::now();
    it_time = init_loop - it_prev;
    it_prev = init_loop;
}

/************************************************************************/
void ThreadImpl::readSensorsFT0()       /** Reading input messages from FT0 SENSORS    **/
{
        //--- FT-Sensor 0 right leg
    Bottle ch0;
    portFt0->read(ch0); // lectura del sensor JR3 ch0 - right foot
    _RF._F.fx = ch0.get(0).asFloat64();
    _RF._F.fy = ch0.get(1).asFloat64();
    _RF._F.fz = ch0.get(2).asFloat64();
    _RF._T.mx = ch0.get(3).asFloat64();
    _RF._T.my = ch0.get(4).asFloat64();
    _RF._T.mz = ch0.get(5).asFloat64();

}

/************************************************************************/
void ThreadImpl::readSensorsFT1()       /** Reading input messages from FT1 SENSORS    **/
{
        //--- FT-Sensor 1 left leg
    Bottle ch1;
    portFt1->read(ch1); // lectura del sensor JR3 ch1 - left foot
    _LF._F.fx = ch1.get(0).asFloat64();
    _LF._F.fy = ch1.get(1).asFloat64();
    _LF._F.fz = ch1.get(2).asFloat64();
    _LF._T.mx = ch1.get(3).asFloat64();
    _LF._T.my = ch1.get(4).asFloat64();
    _LF._T.mz = ch1.get(5).asFloat64();

}

/************************************************************************/
void ThreadImpl::getCurrentTime()       /** Get Current Time    **/
{
    act_time = Time::now() - init_time;
    act_loop = Time::now() - init_loop;
}

/************************************************************************/
void ThreadImpl::zmpCompFT()        /** Calculating ZMP-FT of the body . **/
{
    //ZMP Equations : Double Support - FT

    _xzmp_ft0 = -(((_RF._T.my/10) + e*_RF._F.fx)) / _RF._F.fz; // xzmp0_ft in [m] right foot
    //_yzmp0_ft = -(((_RF._T.mx/10) + e*_RF._F.fy)) / _RF._F.fz; // xzmp0_ft in [m] right foot
    _xzmp_ft1 = -(((_LF._T.my/10) + e*_LF._F.fx)) / _LF._F.fz; // xzmp1_ft in [m] left foot
    //_yzmp1_ft = -(((_LF._T.mx/10) + e*_LF._F.fy)) / _LF._F.fz; // xzmp1_ft in [m] left foot

    _xzmp_ft01 = (_xzmp_ft0 * _RF._F.fz + _xzmp_ft1 * _LF._F.fz) / (_RF._F.fz + _LF._F.fz); // xzmp_ft in [m] robot
    //_yzmp01_ft = (_yzmp0_ft * _RF._F.fz + _yzmp1_ft * _LF._F.fz) / (_RF._F.fz + _LF._F.fz); // xzmp_ft in [m] robot

    // OFFSET FT - eliminando el offset de _xzmp01_ft (ZMP de ambos sensores en X)
    if (n >=1 && n < 50){
        sum_x_ft = _xzmp_ft01 + sum_x_ft;
        offs_x_ft = sum_x_ft / n;
        printf("offs = %f\n", offs_x_ft);
    }

    Xzmp_ft  = _xzmp_ft01 - offs_x_ft; // frontal plane
    //Yzmp_ft = _yzmp01_ft - offs_y_ft; // saggital plane

    if ((_xzmp_ft01 != _xzmp_ft01) || (_yzmp_ft01 != _yzmp_ft01)){
        printf ("Warning: No zmp data\n");
    }
}

/************************************************************************/
void ThreadImpl::evaluateModel()        /** Calculating OUTPUT (Qi) of the legs. **/
{
    // obtaining the angle error for the D-LIPM space state
    _evalLIPM.model(Xzmp_ft,zmp_ref);

    ka = 0.25 * zmp_ref + 9.95; // dudo entre zmp_ref o Xzmp_ft
    _ang_ref = (zmp_ref*(-G))/ (L*(ka-G));
    _ang_out =  _evalLIPM.ang_error_out + _ang_ref;
}

/************************************************************************/
void ThreadImpl::setJoints()        /** Position control **/
{
    rightLegIPositionControl->positionMove(4, _ang_out); // position in degrees
    leftLegIPositionControl->positionMove(4, _ang_out);
}

/************************************************************************/
void ThreadImpl::printData()
{
    cout << endl << "El ZMP REF es: " << zmp_ref << endl;
    cout << endl << "El ZMP FT es: " << Xzmp_ft << endl;
    cout << endl << "El angulo out es: " << _ang_out << endl;
}

/************************************************************************/
void ThreadImpl::saveInFileCsv()
{

    fprintf(fp,"\n%.2f", act_time);

    fprintf(fp,",%.10f", _RF._F.fx); // f_x - sensor ft 0
    fprintf(fp,",%.10f", _RF._F.fz); // f_z - sensor ft 0
    fprintf(fp,",%.10f", _RF._T.my); // m_y - sensor ft 0

    fprintf(fp,",%.10f", _LF._F.fx); // f_x - sensor ft 1
    fprintf(fp,",%.10f", _LF._F.fz); // f_z - sensor ft 1
    fprintf(fp,",%.10f", _LF._T.my); // m_y - sensor ft 1

    fprintf(fp,",%.8f", Xzmp_ft); // ZMP body (double support) (frontal plane)
    fprintf(fp,",%.8f", _xzmp_ft0); // zmp (right foot)
    fprintf(fp,",%.8f", _xzmp_ft1); // zmp (leftt foot)

    fprintf(fp,",%i", n);

}

/************************************************************************/
void ThreadImpl::setIEncodersControl(IEncoders *iRightLegEncoders, IEncoders *iLeftLegEncoders)
{
    this->rightLegIEncoders = iRightLegEncoders;
    this->leftLegIEncoders = iLeftLegEncoders;
}

/************************************************************************/
void ThreadImpl::setIPositionControl(IPositionControl *iRightLegPositionControl,IPositionControl *iLeftLegPositionControl)
{
    this->rightLegIPositionControl = iRightLegPositionControl;
    this->leftLegIPositionControl = iLeftLegPositionControl;
}

/************************************************************************/
void ThreadImpl::setInputPorts(Port *inputPortFt0,Port *inputPortFt1)
{
    this->portFt0 = inputPortFt0;
    this->portFt1 = inputPortFt1;
}

} // namespace roboticslab
