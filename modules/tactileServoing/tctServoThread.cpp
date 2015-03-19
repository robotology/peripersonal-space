#include "tctServoThread.h"
#include <fstream>
#include <sstream>
#include <iomanip>

#define HAND_LEFT      1
#define FOREARM_LEFT   2
#define HAND_RIGHT     4
#define FOREARM_RIGHT  5
#define VEL_THRES      0.000001        // m/s?
// VEL_THRES * getRate()

tactileServoThread::tactileServoThread(int _rate, const string &_name, const string &_robot, int _v) :
                                     RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    armR = new iCubArm("right");
    armL = new iCubArm("left");

    fingerL = new iCubFinger("left_index");
    fingerR = new iCubFinger("right_index");

    ft_vector.resize(6,0.0);
    ft_vector_filter.resize(6,0.0);
    ft_vector_old.resize(6,0.0);
    ft_avg.resize(6,0.0);

    //get the left forearm vector which is desired vector to let the right hand index finger following
    DesVec.resize(3,0.0);

    //init desired force and position and force control
    Desf.resize(3,0.0);

    PCtrl.resize(3,3);

    movementCnt = 0;
}

bool tactileServoThread::threadInit()
{
    ft_inR.open(("/"+name +"/right_arm/ft:i").c_str());
    // ft_inL.open(("/"+name +"/left_arm/ft:i").c_str());

    if (robot == "icub")
    {
        yarp::os::Network::connect("/wholeBodyDynamics/right_arm/endEffectorWrench:o",("/"+name +"/right_arm/ft:i").c_str());
        // yarp::os::Network::connect("/wholeBodyDynamics/left_arm/endEffectorWrench:o",("/"+name +"/left_arm/ft:i").c_str());
    }
    
    if (!calibrateFTReadings())
    {
        return false;
    }

    Property OptR;
    OptR.put("robot",  robot.c_str());
    OptR.put("part",   "right_arm");
    OptR.put("device", "remote_controlboard");
    OptR.put("remote",("/"+robot+"/right_arm").c_str());
    OptR.put("local", ("/"+name +"/right_arm").c_str());
    if (!ddR.open(OptR))
    {
        printMessage(0,"ERROR: could not open right_arm PolyDriver!\n");
        return false;
    }

    Property OptL;
    OptL.put("robot",  robot.c_str());
    OptL.put("part",   "left_arm");
    OptL.put("device", "remote_controlboard");
    OptL.put("remote",("/"+robot+"/left_arm").c_str());
    OptL.put("local", ("/"+name +"/left_arm").c_str());
    if (!ddL.open(OptL))
    {
        printMessage(0,"ERROR: could not open left_arm PolyDriver!\n");
        return false;
    }

    Property OptT;
    OptT.put("robot",  robot.c_str());
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote",("/"+robot+"/torso").c_str());
    OptT.put("local", ("/"+name +"/torso").c_str());
    if (!ddT.open(OptT))
    {
        printMessage(0,"ERROR: could not open torso PolyDriver!\n");
        return false;
    }

    Property optionCartL;
    optionCartL.put("device","cartesiancontrollerclient");
    optionCartL.put("remote","/"+robot+"/cartesianController/left_arm");
    optionCartL.put("local",("/"+name+"/cart/left_arm").c_str());
    if (!dcartL.open(optionCartL))
    {
        printMessage(0,"ERROR: could not open left_arm CartesianControllerClient!\n");
        return false;
    }

    Property optionCartR;
    optionCartR.put("device","cartesiancontrollerclient");
    optionCartR.put("remote","/"+robot+"/cartesianController/right_arm");
    optionCartR.put("local",("/"+name+"/cart/right_arm").c_str());
    if (!dcartR.open(optionCartR))
    {
        printMessage(0,"ERROR: could not open right_arm CartesianControllerClient!\n");
        return false;
    }

    Property OptGaze;
    OptGaze.put("device","gazecontrollerclient");
    OptGaze.put("remote","/iKinGazeCtrl");
    OptGaze.put("local",("/"+name+"/gaze").c_str());

    if ((!ddG.open(OptGaze)) || (!ddG.view(igaze))){
       printMessage(0,"Error: could not open the Gaze Controller!\n");
       return false;
    }
    igaze -> storeContext(&contextGaze);
    igaze -> setSaccadesStatus(false);
    igaze -> setNeckTrajTime(0.75);
    igaze -> setEyesTrajTime(0.5);

    // open the view
    dcartL.view(icartL);
    icartL->storeContext(&startup_left_context_id);
    icartL->setTrajTime(1.0);
    
    // open the view
    dcartR.view(icartR);
    icartR->storeContext(&startup_right_context_id);
    icartR->setTrajTime(1.0);
    
    bool ok = 1;

    if (ddL.isValid())
    {
        ok = ok && ddL.view(iencsL);
        ok = ok && ddL.view(ilimL);
        // ok = ok && ddL.view(iposL);
        // ok = ok && ddL.view(imodeL);
        // ok = ok && ddL.view(iimpL);
    }
    iencsL->getAxes(&jntsL);
    encsL = new Vector(jntsL,0.0);

    if (ddR.isValid())
    {
        ok = ok && ddR.view(iencsR);
        ok = ok && ddR.view(ilimR);
        // ok = ok && ddR.view(iposR);
        // ok = ok && ddR.view(imodeR);
        // ok = ok && ddR.view(iimpR);
    }
    iencsR->getAxes(&jntsR);
    encsR = new Vector(jntsR,0.0);

    if (ddT.isValid())
    {
        ok = ok && ddT.view(iencsT);
        ok = ok && ddT.view(ilimT);
    }
    iencsT->getAxes(&jntsT);
    encsT = new Vector(jntsT,0.0);

    // Align joints bounds for the two iCubArms
    deque<IControlLimits*> limL;
    limL.push_back(ilimT);
    limL.push_back(ilimL);
    armL->alignJointsBounds(limL);

    deque<IControlLimits*> limR;
    limR.push_back(ilimT);
    limR.push_back(ilimR);
    armR->alignJointsBounds(limR);

    // Let's attach the homogeneous transfor matrixes of the fingers to the arms
    Vector jointsL;
    iencsL  -> getEncoders(encsL->data());
    fingerL -> getChainJoints(*encsL,jointsL);

    Matrix HIndexL = fingerL -> getH(jointsL*CTRL_DEG2RAD);
    // Matrix HIndexL = eye(4);
    armL -> setHN(HIndexL);
    printMessage(1,"HIndexL:\n%s\n", HIndexL.toString(3,3).c_str());

    Vector jointsR;
    iencsR  -> getEncoders(encsR->data());
    fingerR -> getChainJoints(*encsR,jointsR);

    Matrix HIndexR = fingerR -> getH(jointsR*CTRL_DEG2RAD);
    // Matrix HIndexR = eye(4);
    armR -> setHN(HIndexR);
    printMessage(1,"HIndexR:\n%s\n", HIndexR.toString(3,3).c_str());

    Vector indexPR = HIndexR.subcol(0,3,3);
    Vector indexOR = iCub::ctrl::dcm2axis(HIndexR);
    icartR -> attachTipFrame(indexPR,indexOR);


    //we are assume to use the right finger to touch left arm, 
    //firstly we get the init pose of the right palm
    initPose=eye(4);
    currPose=eye(4);
    getCurPose();

    initPose        = currPose;
    position_0_g    = initPose.subcol(0,3,3);
    orientation_0_g = iCub::ctrl::dcm2axis(initPose);
    
    Desf[0] =  0.0;
    Desf[1] =  0.0;
    Desf[2] = -1.0;

    PCtrl(0,0) = -0.01;
    PCtrl(1,1) = -0.01;
    PCtrl(2,2) = -0.01;

    return true;
}

bool tactileServoThread::calibrateFTReadings()
{
    Vector ft_sum(6,0.0);

    for (size_t i = 0; i < 100; i++)
    {
        Bottle *ftbottle = ft_inR.read(true);
        for(size_t j = 0; j<6; j++)
        {
            ft_vector[j] = ftbottle->get(j).asDouble();
        }
        printMessage(3,"Calibration Value #%i:\t%s\n",i,ft_vector.toString(3,3).c_str());
        ft_sum=ft_sum+ft_vector;
    }

    ft_avg=ft_sum/100;
    printMessage(1,"Calibrated Force value Value:\t%s\n",ft_avg.toString(3,3).c_str());

    if (ft_sum[0] == 0.0 && ft_sum[1] == 0.0 && ft_sum[2] == 0.0 &&
        ft_sum[3] == 0.0 && ft_sum[4] == 0.0 && ft_sum[5] == 0.0)
    {
        printMessage(0,"ERROR! FT sensor is not working!\n");
        return false;
    }

    return true;
}

Vector tactileServoThread::getNewDesVec(const string &arm)
{
    if (arm == "left_arm")
    {
        Matrix HElbow= armL -> getH(6, true);
        Matrix HWrist= armL -> getH(7, true);

        Vector posElbow = HElbow.subcol(0,3,3);
        Vector posWrist = HWrist.subcol(0,3,3);

        Vector newDesVec = posWrist - posElbow;
        newDesVec = newDesVec/norm(newDesVec);

        printMessage(0,"newDesVec: %s\n",newDesVec.toString(3,3).c_str());
        return newDesVec;
    }
}

bool tactileServoThread::getFT()
{
    Bottle *ftbottle = ft_inR.read(false);

    if(ftbottle != 0) //else ft_vector keeps its old value 
    {
        for(size_t i = 0; i<6; i++)
        {
            ft_vector[i] = ftbottle->get(i).asDouble();
        }
        printMessage(2,"Raw      Force value is %s\n",ft_vector.toString(3,3).c_str());
    }
    else
    {
        printMessage(0,"Can not read data this step\n");
    }

    if((ft_vector[0] != 0.0) && (ft_vector[1] != 0.0) && (ft_vector[2] != 0.0) &&
       (ft_vector[3] != 0.0) && (ft_vector[4] != 0.0) && (ft_vector[5] != 0.0))
    { 
        ft_vector=ft_vector-ft_avg;
        printMessage(1,"Avg      Force value is %s\n",ft_vector.toString(3,3).c_str());

        ft_vector_filter = 0.95 * ft_vector + 0.05 * ft_vector_old;
        ft_vector_old = ft_vector;
        printMessage(1,"Filtered Force value is %s\n",ft_vector_filter.toString(3,3).c_str());
    }
    else 
    {
        printMessage(0,"ERROR!! Force Torque sensor is screwing things up!\n");
        return false;
    }
      
    if(fabs(ft_vector_filter[3])<0.8)
    {
        ft_vector_filter[3] = 0;
    }

    if(fabs(ft_vector_filter[4])<0.8)
    {
        ft_vector_filter[4] = 0;
    }

    if(fabs(ft_vector_filter[5])<0.1)
    {
        ft_vector_filter[5] = 0;
    }

    return true;
}

void tactileServoThread::getCurPose()
{
    updateChains();
    currPose     = armR -> getH();
    position_0_g = currPose.subcol(0,3,3);
}

Vector tactileServoThread::getnewposition(Vector desv)
{
    Vector devf(3,0.0);
    Vector desvl(3,0.0);
    Vector desvg(3,0.0);
    Vector newp(3,0.0);
    Matrix tmpm(3,3);
    Matrix tmpm_inv(3,3);
    
    tmpm     = currPose.submatrix(0,2,0,2);
    tmpm_inv = tmpm.transposed();
    printMessage(4,"tmpm_inv is \n%s\n",tmpm_inv.toString(3,3).c_str());

    //in the local frame force deviation devf;
    devf[2] = -0.02*(Desf[2] - ft_vector_filter[2]);
    printMessage(0,"DevF is: %s\n",devf.toString(3,3).c_str());

    //compute the desired velocity along the desired vector provided by fore-arm vector
    printMessage(2,"desv is: %s\n",desv.toString(3,3).c_str());
    // desvl = tmpm_inv * 0.01 * desv;

    int cycle=10;
    if (movementCnt<cycle)
    {
        desvl[1] = 0.02;
        movementCnt++;
    }
    else if (movementCnt<2*cycle)
    {
        desvl[1] = -0.02;
        movementCnt++;
    }
    else if (movementCnt==2*cycle)
    {
        desvl[1] = 0.00;
        movementCnt = 0;
    }

    desvl[0] = 0.0;
    desvl[2] = devf[2];
    desvg = tmpm * desvl;

    printMessage(0,"DesVg is: %s\n",desvg.toString(3,3).c_str());

    for(int i = 0; i < 3; i++)
    {
        newp[i] = position_0_g[i] + desvg[i];
    }

    printMessage(1,"OldP is: %s\n",position_0_g.toString(3,3).c_str());
    printMessage(1,"NewP is: %s\n",newp.toString(3,3).c_str());

    return newp;
}

void tactileServoThread::run()
{
    printf("\n");
    Vector newp;

    if (getFT())
    {
        getCurPose();
        DesVec =  getNewDesVec("left_arm");
        newp   =  getnewposition(DesVec);
        icartR -> goToPose(newp,orientation_0_g);
        igaze  -> lookAtFixationPoint(newp);
    }
}

void tactileServoThread::updateChains()
{
    /**
    * ENCODERS LEFT (They're 3 DOF for the torso (always 0) + 7 DOF straightforwardly acquired from shoulder to wrist)
    */
    iencsL->getEncoders(encsL->data());
    // Vector qL(10,0.0);
    // qL.setSubvector(3,encsL->subVector(0,6));
    // armL -> setAng(qL*CTRL_DEG2RAD);
    armL -> setAng(encsL->subVector(0,6)*CTRL_DEG2RAD);
    printMessage(4,"ArmL has been set up to %s\n",(armL -> getAng()*CTRL_RAD2DEG).toString(3,3).c_str());

    /**
    * ENCODERS RIGHT (They're 3 DOF for the torso (always 0) + 7 DOF straightforwardly acquired from shoulder to wrist)
    */
    iencsR->getEncoders(encsR->data());
    // Vector qR(10,0.0);
    // qR.setSubvector(3,encsR->subVector(0,6));
    // armR -> setAng(qR*CTRL_DEG2RAD);
    armR -> setAng(encsR->subVector(0,6)*CTRL_DEG2RAD);
    printMessage(4,"ArmR has been set up to %s\n",(armR -> getAng()*CTRL_RAD2DEG).toString(3,3).c_str());
}

int tactileServoThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"*** %s: ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void tactileServoThread::threadRelease()
{
    printMessage(0,"Deleting arms..\n");
        if (armL)
        {
            delete armL;
            armL = NULL;
        }
        if (armR)
        {
            delete armR;
            armR = NULL;
        }
        if (fingerL)
        {
            delete fingerL;
            fingerL = NULL;
        }
        if (fingerR)
        {
            delete fingerR;
            fingerR = NULL;
        }

    printMessage(0,"Closing cartesian stuff..\n");
        // we require an immediate stop
        // before closing the client for safety reason
        icartL->stopControl();
        // it's a good rule to restore the controller
        // context as it was before opening the module
        icartL->restoreContext(startup_left_context_id);
        dcartL.close();
	
        icartR->stopControl();
        icartR->restoreContext(startup_right_context_id);
        dcartR.close();

        ddL.close();
        ddR.close();

        Vector ang(3,0.0);
        igaze -> lookAtAbsAngles(ang);
        igaze -> restoreContext(contextGaze);
        igaze -> stopControl();
}

// empty line to make gcc happy
