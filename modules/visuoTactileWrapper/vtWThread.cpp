#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "vtWThread.h"

vtWThread::vtWThread(int _rate, const string &_name, const string &_robot, int _v) :
                       RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    motionCUTPos.resize(3,0.0);
    motionCUTVelEstimate.resize(3,0.0);

    pf3dTrackerPos.resize(3,0.0);
    pf3dTrackerVelEstimate.resize(3,0.0);

    doubleTouchPos.resize(3,0.0);
    doubleTouchVelEstimate.resize(3,0.0);
    
    armR = new iCubArm("right");
    armL = new iCubArm("left");

    doubleTouchStep =   -1;
}

bool vtWThread::threadInit()
{
    motionCUTPort.open(("/"+name+"/motionCUT:i").c_str());
    pf3dTrackerPort.open(("/"+name+"/pf3dTracker:i").c_str());
    doubleTouchPort.open(("/"+name+"/doubleTouch:i").c_str());
    eventsPort.open(("/"+name+"/events:o").c_str());
    depth2kinPort.open(("/"+name+"/depth2kin:o").c_str());
    
    Network::connect("/motionCUT/blobs:o",("/"+name+"/motionCUT:i").c_str());
    Network::connect("/pf3dTracker/data:o",("/"+name+"/pf3dTracker:i").c_str());
    Network::connect("/doubleTouch/status:o",("/"+name+"/doubleTouch:i").c_str());
    Network::connect(("/"+name+"/events:o").c_str(),"/visuoTactileRF/events:i");

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
    
    /**************************/
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
        bool ok = 1;
        if (ddR.isValid())
        {
            ok = ok && ddR.view(iencsR);
        }
        if (!ok)
        {
            printMessage(0,"\nERROR: Problems acquiring right_arm interfaces!!!!\n");
            return false;
        }
        iencsR->getAxes(&jntsR);
        encsR = new yarp::sig::Vector(jntsR,0.0);

    /**************************/
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
        ok = 1;
        if (ddL.isValid())
        {
            ok = ok && ddL.view(iencsL);
        }
        if (!ok)
        {
            printMessage(0,"\nERROR: Problems acquiring left_arm interfaces!!!!\n");
            return false;
        }
        iencsL->getAxes(&jntsL);
        encsL = new yarp::sig::Vector(jntsL,0.0);

    linEst_motionCUT   = new AWLinEstimator(16,0.05);
    linEst_pf3dTracker = new AWLinEstimator(16,0.05);
    linEst_doubleTouch = new AWLinEstimator(16,0.05);
    
    return true;
}

void vtWThread::run()
{
    bool isTarget = false;
    events.clear();

    // process the motionCUT
    if (motionCUTBottle = motionCUTPort.read(false))
    {
        if (motionCUTBottle!=NULL)
        {
            motionCUTPos.zero();
            int maxSize = -1;
            int blobidx = -1;
            int size    = -1;
            Bottle *blob;

            // Let's process the blob with the maximum size
            blob = motionCUTBottle->get(blobidx).asList();

            Vector px(2,0.0);
            px(0) = blob->get(0).asInt();
            px(1) = blob->get(1).asInt();

            if (SFMPort.getOutputCount()>0)
            {
                Bottle cmd,reply;
                cmd.addInt(px[0]);
                cmd.addInt(px[1]);

                SFMPort.write(cmd,reply);
                motionCUTPos[0]=reply.get(1).asDouble();
                motionCUTPos[1]=reply.get(2).asDouble();
                motionCUTPos[2]=reply.get(3).asDouble();
            }
            else
            {
                // 0 is for the left image
                // 1 is the distance [m] of the object from the image plane (extended to infinity)
                igaze->get3DPoint(0,px,1,motionCUTPos);
            }

            if (motionCUTPos[0]!=0.0 && motionCUTPos[1]!=0.0 && motionCUTPos[2]!=0.0)
            {
                if (depth2kinPort.getOutputCount()>0)
                {
                    Bottle cmd,reply;
                    cmd.addString("getPoint");
                    cmd.addString("right");   // TO BE MODIFIED
                    cmd.addDouble(motionCUTPos[0]);
                    cmd.addDouble(motionCUTPos[1]);
                    cmd.addDouble(motionCUTPos[2]);

                    depth2kinPort.write(cmd,reply);
                    motionCUTPos[0]=reply.get(1).asDouble();
                    motionCUTPos[1]=reply.get(2).asDouble();
                    motionCUTPos[2]=reply.get(3).asDouble();
                }

                AWPolyElement el(motionCUTPos,Time::now());
                motionCUTVelEstimate=linEst_motionCUT->estimate(el);
                
                events.push_back(IncomingEvent(motionCUTPos,motionCUTVelEstimate,0.05,"motionCUT"));
                isTarget=true;
            }
        }
    }

    // process the pf3dTracker
    if (pf3dTrackerBottle = pf3dTrackerPort.read(false))
    {
        if (pf3dTrackerBottle->size()>6)
        {
            if (pf3dTrackerBottle->get(6).asDouble()==1.0)
            {
                Vector fp(4);
                fp[0]=pf3dTrackerBottle->get(0).asDouble();
                fp[1]=pf3dTrackerBottle->get(1).asDouble();
                fp[2]=pf3dTrackerBottle->get(2).asDouble();
                fp[3]=1.0;

                if (!gsl_isnan(fp[0]) && !gsl_isnan(fp[1]) && !gsl_isnan(fp[2]))
                {
                    printMessage(1,"Computing data from the pf3dTracker %g\n",getEstUsed());
                    Vector x,o;
                    igaze->getLeftEyePose(x,o);
                    
                    Matrix T=axis2dcm(o);
                    T(0,3)=x[0];
                    T(1,3)=x[1];
                    T(2,3)=x[2];

                    pf3dTrackerPos=T*fp;
                    pf3dTrackerPos.pop_back();
                    
                    AWPolyElement el(pf3dTrackerPos,Time::now());
                    pf3dTrackerVelEstimate=linEst_pf3dTracker->estimate(el);
                    
                    events.push_back(IncomingEvent(pf3dTrackerPos,pf3dTrackerVelEstimate,0.05,"pf3dTracker"));
                    isTarget=true;
                }
            }
        }
    }

    // process the doubleTouch
    if(doubleTouchBottle = doubleTouchPort.read(false))
    {
        if(doubleTouchBottle != NULL)
        {
            if (doubleTouchBottle->get(3).asString() != "")
            {
                Matrix T = eye(4);
                Vector fingertipPos(4,0.0), fingertipPosWRF(4,0.0);
                
                currentTask = doubleTouchBottle->get(3).asString();
                doubleTouchStep = doubleTouchBottle->get(0).asInt();
                fingertipPos = matrixFromBottle(*doubleTouchBottle,20,4,4).subcol(0,3,3); // fixed translation from the palm
                fingertipPos.push_back(1.0);

                if((doubleTouchStep>3) && (doubleTouchStep<7))
                {
                    if(currentTask=="R2L") //right to left -> the right index finger will be generating events
                    { 
                        iencsR->getEncoders(encsR->data());
                        Vector qR=encsR->subVector(0,6);
                        armR -> setAng(qR*CTRL_DEG2RAD);                        
                        T = armR -> getH(3+6, true);  // torso + up to wrist
                        fingertipPosWRF = T * fingertipPos; 
                        //optionally, get the finger encoders and get the fingertip position using iKin Finger based on the current joint values 
                        //http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html#sec_cart_tipframe
                        fingertipPosWRF.pop_back(); //take out the last dummy value from homogenous form
                    }
                    else if(currentTask=="L2R") //left to right -> the left index finger will be generating events
                    {   
                        iencsL->getEncoders(encsL->data());
                        Vector qL=encsL->subVector(0,6);
                        armL -> setAng(qL*CTRL_DEG2RAD);                        
                        T = armL -> getH(3+6, true);  // torso + up to wrist
                        fingertipPosWRF = T * fingertipPos; 
                        //optionally, get the finger encoders and get the fingertip position using iKin Finger based on the current joint values 
                        //http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html#sec_cart_tipframe
                        fingertipPosWRF.pop_back(); //take out the last dummy value from homogenous form
                    } 
                    else
                    {
                        printMessage(0,"\nERROR: vtWThread::run(): Unknown task received from double touch thread!\n");
                    }
                                   
                    AWPolyElement el2(fingertipPosWRF,Time::now());
                    doubleTouchVelEstimate=linEst_doubleTouch->estimate(el2);
                    events.push_back(IncomingEvent(fingertipPosWRF,doubleTouchVelEstimate,-1.0,"doubleTouch"));
                    isTarget=true;
                }
            }
        }
    }
    
    if (isTarget)
    {
        igaze -> lookAtFixationPoint(pf3dTrackerPos);

        Bottle& eventsBottle = eventsPort.prepare();
        eventsBottle.clear();
        for (size_t i = 0; i < events.size(); i++)
        {
            eventsBottle.addList()= events[i].toBottle();
        }
        eventsPort.write();
    }
}

int vtWThread::printMessage(const int l, const char *f, ...) const
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

void vtWThread::threadRelease()
{
    printMessage(0,"Closing gaze controller..\n");
        Vector ang(3,0.0);
        igaze -> lookAtAbsAngles(ang);
        igaze -> restoreContext(contextGaze);
        igaze -> stopControl();
        ddG.close();

    printMessage(0,"Closing ports..\n");
        motionCUTPort.interrupt();
        motionCUTPort.close();
        printMessage(1,"motionCUTPort successfully closed!\n");
        pf3dTrackerPort.interrupt();
        pf3dTrackerPort.close();
        printMessage(1,"pf3dTrackerPort successfully closed!\n");
        doubleTouchPort.interrupt();
        doubleTouchPort.close();
        printMessage(1,"doubleTouchPort successfully closed!\n");
        eventsPort.interrupt();
        eventsPort.close();
        printMessage(1,"eventsPort successfully closed!\n");
        depth2kinPort.interrupt();
        depth2kinPort.close();
        printMessage(1,"depth2kinPort successfully closed!\n");
}

// empty line to make gcc happy
